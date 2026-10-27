"""Read-only raw-localization shadow planner; no reference output is fed back as a sensor or command."""
import json
import math
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from intramural_mission.avoidance import PlanningWorker, preview
from intramural_mission.trajectory import OmniTracker


def recorded_body_pose(message, lidar_pose):
    """Match the current loam_interface + sensor_scan_generation frame composition."""
    from scipy.spatial.transform import Rotation
    if message.header.frame_id!='camera_init' or message.child_frame_id!='body':
        raise ValueError('Expected recorded Point-LIO camera_init -> body odometry')
    p,q=message.pose.pose.position,message.pose.pose.orientation
    mount=np.eye(4);mount[:3,:3]=Rotation.from_euler('xyz',lidar_pose[3:]).as_matrix()
    mount[:3,3]=lidar_pose[:3]
    pose=np.eye(4);pose[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    pose[:3,3]=[p.x,p.y,p.z]
    body=mount@pose@np.linalg.inv(mount)
    if not np.isfinite(body).all():raise ValueError('Nonfinite recorded body pose')
    return [float(body[0,3]),float(body[1,3]),math.atan2(body[1,0],body[0,0])]


class RawShadow:
    def __init__(self,node,config,grid,plan,stamps,output):
        self.node,self.c,self.plan,self.stamps,self.output=node,config,plan,np.asarray(stamps),output
        self.worker=PlanningWorker(config,grid)
        self.observer=OmniTracker(plan,config)
        self.buffer=Buffer(node=node);self.listener=TransformListener(self.buffer,node)
        self.publisher=node.create_publisher(String,'mission/trajectory',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(String,'motion_guard/observations',self.receive,1)
        self.latest=None;self.submitted=-math.inf;self.last_snapshot=-math.inf
        self.pose=None;self.pose_age=math.inf;self.progress=0.
        self.run_id='raw-shadow-'+output.name
        self.decisions=[];self.poses=[]
        self.counts=dict(snapshots=0,obstacle_updates=0,planner_reports=0)
        self.timer=node.create_timer(.025,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self,message):
        data=json.loads(message.data)
        if self.latest is None or data['stamp']>self.latest['stamp']:
            self.latest=data;self.counts['obstacle_updates']+=1

    def tick(self):
        now=self.node.get_clock().now().nanoseconds/1e9
        result=self.worker.poll()
        if result is not None:
            proposal=result.pop('plan');serialized=result.pop('plan_json',None)
            result.update(has_proposal=proposal is not None,received_stamp=now,
                observation_to_result_seconds=now-result['stamp'],execution_permitted=False,
                would_reject_deadline=result['seconds']>.08 or now-result['stamp']>.5)
            if proposal is not None:
                name=f'raw-shadow-route-{len(self.decisions):03}.json'
                (self.output/name).write_text(serialized+'\n')
                result['proposal_file']=name
            self.decisions.append(result);self.counts['planner_reports']+=1
        try:
            transform=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
        except TransformException:
            return
        source=transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
        if abs(now-source)>.25:return
        p,q=transform.transform.translation,transform.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose=[p.x,p.y];self.pose_age=now-source
        self.observer.observe(self.pose,.025,source)
        # Recorded time indexes the reference, while all localization state comes from the raw replay graph.
        self.progress=float(np.interp(now,self.stamps,self.plan.s))
        if now>self.last_snapshot:
            points,times=preview(self.plan,self.progress,float(np.linalg.norm(self.observer.velocity)))
            self.publisher.publish(String(data=json.dumps(dict(stamp=now,run_id=self.run_id,state='RUNNING',phase='RETURN',
                route_id=self.plan.route_id,pose=[p.x,p.y,yaw],pose_stamp=source,
                command=[0.,0.],points=points.tolist(),times=times.tolist()))))
            self.last_snapshot=now;self.counts['snapshots']+=1
            self.poses.append([now,source,p.x,p.y,yaw])
        if self.latest is not None and self.latest['stamp']>self.submitted:
            data=self.latest;self.submitted=data['stamp']
            self.worker.submit(dict(run_id=self.run_id,plan=self.plan,original_plan=self.plan,phase=0,
                progress=self.progress,pose=self.pose,pose_age=self.pose_age,
                velocity=self.observer.velocity.tolist(),command=self.observer.velocity.tolist(),acceleration=[0.,0.],
                stamp=data['stamp'],source_stamp=data['source_stamp'],point_offsets=data.get('point_offsets'),
                sensor_position=data.get('sensor_position'),points=data['points'],weights=data['weights'],
                observation_age=max(0.,now-data['stamp'])))

    def close(self):
        self.worker.close()
        (self.output/'raw-shadow-decisions.jsonl').write_text(''.join(json.dumps(d)+'\n'for d in self.decisions))
        (self.output/'raw-shadow-localized-poses.json').write_text(json.dumps(self.poses)+'\n')

    def report(self):
        return dict(passed=self.counts['obstacle_updates']>=5 and self.counts['planner_reports']>=5,
            counts=self.counts,proposals=sum(d['has_proposal']for d in self.decisions),
            predicted_conflicts=sum(d['blocked']for d in self.decisions),
            scope='1x raw sensors -> live localization TF -> production guard observations -> production planning worker; zero transmitted commands',
            reference_scope='Single RETURN geometry indexed by recorded time; not historical ordered mission progress or closed-loop detour execution',
            command_assumption='Unrecorded active command approximated by raw-localized velocity only inside predictions; emitted snapshot command is always zero')
