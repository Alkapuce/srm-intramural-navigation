"""Exercise the real CLI against an isolated ROS mission service and status stream."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]


class MissionConsoleTests(unittest.TestCase):
    def scenario(self, outcome, interrupt=False, busy=False, planning_delay=0., status_delay=0., operation='start'):
        rclpy.init(domain_id=97)
        node=rclpy.create_node('external_mission_service')
        pub=node.create_publisher(String, '/intramural_robot/mission/status',
                                  QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        status=dict(state='RUNNING' if busy else 'DONE',phase='RETURN',segment=9,run=1,
                    run_id='old',reason='',history=[],pose=[0.,0.],blocked_count=0,
                    elapsed_sim_seconds=42.,tracking={})
        counts=dict(start=0,reset=0,cancel=0)
        began=[None]
        def reset(req,res):
            counts['reset']+=1;status.update(state='IDLE');res.success=True;return res
        def start(req,res):
            counts['start']+=1
            if busy:
                res.message='Mission already running';return res
            time.sleep(planning_delay)
            began[0]=time.monotonic()
            status.update(state='RUNNING',phase='ENTER',segment=0,run=2,run_id='console-test',
                          elapsed_sim_seconds=0.,history=[dict(sim_time=1792421431.328,segment=0)])
            res.success=True;res.message='Mission started';return res
        def cancel(req,res):
            counts['cancel']+=1;status.update(state='CANCELLED');res.success=True;res.message='Cancellation latched';return res
        def prepare(req,res):
            counts['prepare']=counts.get('prepare',0)+1;status.update(state='PREPARED')
            res.success=True;res.message='Route prepared';return res
        def publish():
            if began[0] is not None and status['state']=='RUNNING':
                elapsed=time.monotonic()-began[0]
                if elapsed < status_delay:return
                status.update(elapsed_sim_seconds=elapsed,pose=[elapsed*.2,0.],tracking=dict(error=.012))
                if elapsed>.6 and not interrupt:
                    status.update(state=outcome,phase='RETURN',segment=9,
                                  reason='STATIC_FOOTPRINT_CLEARANCE' if outcome=='FAULT' else '')
            pub.publish(String(data=json.dumps(status)))
        for name,callback in [('reset',reset),('start',start),('cancel',cancel),('prepare',prepare)]:
            node.create_service(Trigger,'/intramural_robot/mission/'+name,callback)
        node.create_timer(.025,publish)
        proc=subprocess.Popen([str(ROOT/'scripts/srm-nav'),operation],cwd=ROOT,
                              env=dict(os.environ,INTRAMURAL_NATIVE='1',INTRAMURAL_ROS_DOMAIN_ID='97'),
                              stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        sent=False
        try:
            deadline=time.monotonic()+planning_delay+10
            while proc.poll() is None and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
                if interrupt and began[0] is not None and time.monotonic()-began[0]>.3 and not sent:
                    proc.send_signal(signal.SIGINT);sent=True
            self.assertIsNotNone(proc.poll(),'CLI did not finish after its terminal status')
            stdout,stderr=proc.communicate(timeout=2)
            return proc.returncode,stdout+stderr,counts
        finally:
            if proc.poll() is None:
                proc.kill();proc.wait()
            node.destroy_node();rclpy.shutdown()

    def test_done_auto_reset_and_progress_finish_in_same_terminal(self):
        code,out,counts=self.scenario('DONE')
        self.assertEqual(code,0,out)
        self.assertEqual(counts,dict(start=1,reset=1,cancel=0))
        for text in ('RUNNING','结果：DONE','总计时：','最大跟踪误差：','开始：','结束：'):
            self.assertIn(text,out)

    def test_prepare_cli_finishes_without_starting_or_cancelling(self):
        code,out,counts=self.scenario('DONE',operation='prepare')
        self.assertEqual(code,0,out)
        self.assertEqual(counts,dict(start=0,reset=1,cancel=0,prepare=1))
        self.assertIn('准备完成',out)

    def test_fault_prints_reason_and_busy_rejection_does_not_cancel_existing_run(self):
        code,out,counts=self.scenario('FAULT')
        self.assertEqual(code,2,out);self.assertIn('STATIC_FOOTPRINT_CLEARANCE',out)
        self.assertEqual(counts['cancel'],0)
        code,out,counts=self.scenario('DONE',busy=True)
        self.assertEqual(code,2,out);self.assertIn('Mission already running',out)
        self.assertEqual(counts,dict(start=1,reset=0,cancel=0))

    def test_ctrl_c_requests_cancellation(self):
        code,out,counts=self.scenario('DONE',interrupt=True)
        self.assertEqual(code,130,out);self.assertEqual(counts['cancel'],1,out)
        self.assertIn('Ctrl-C',out);self.assertIn('Cancellation latched',out)

    def test_planning_gap_is_not_a_running_status_failure(self):
        code,out,counts=self.scenario('DONE',planning_delay=2.5,status_delay=.15)
        self.assertEqual(code,0,out)
        self.assertEqual(counts,dict(start=1,reset=1,cancel=0))

    def test_long_planning_response_does_not_hit_the_ordinary_service_timeout(self):
        code,out,counts=self.scenario('DONE',planning_delay=30.5,status_delay=.15)
        self.assertEqual(code,0,out)
        self.assertEqual(counts,dict(start=1,reset=1,cancel=0))
        self.assertIn('已等待',out)

    def test_missing_status_after_start_still_requests_cancellation(self):
        code,out,counts=self.scenario('DONE',status_delay=2.5)
        self.assertEqual(code,2,out)
        self.assertEqual(counts['cancel'],1,out)

    def test_saved_result_stops_at_first_terminal_status(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
            folder=Path(temp)
            history=[dict(sim_time=1792421431.328,segment=0)]
            rows=[dict(state='RUNNING',elapsed_sim_seconds=0.,history=history,pose=[0.,0.],tracking={}),
                  dict(state='RUNNING',elapsed_sim_seconds=26.,history=history,pose=[.06,0.],tracking=dict(error=.108)),
                  dict(state='DONE',elapsed_sim_seconds=42.64,history=history,pose=[.04,0.],tracking={}),
                  dict(state='DONE',elapsed_sim_seconds=42.64,history=history,pose=[5.,0.],tracking={})]
            (folder/'status.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            (folder/'manifest.json').write_text(json.dumps(dict(config=dict(parking=[0.,0.]))))
            result=subprocess.run([str(ROOT/'scripts/srm-nav'),'result',str(folder)],text=True,capture_output=True,timeout=3)
            self.assertEqual(result.returncode,0,result.stderr)
            for text in ('42.64','26.00','16.64','0.108','0.040','22:51:13.968'):
                self.assertIn(text,result.stdout)


if __name__=='__main__':
    unittest.main()
