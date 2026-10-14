#!/usr/bin/env python3
"""Persist live TF authorities, node clock settings and graph interfaces."""
import argparse
import json
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',default='/work/artifacts/stage2/navigation-complete')
args=parser.parse_args()
out=Path(args.output).resolve()
if not out.is_relative_to(Path('/work/artifacts')): raise ValueError('Invalid output')
out.mkdir(parents=True,exist_ok=True)
rclpy.init()
node=rclpy.create_node('stage2_graph_inspector')
ns='/red_standard_robot1'
try:
    import subprocess
    binary=Path('/work/cache/stage2/tf-authorities')
    source=Path('/work/scripts/validation/tf-authorities.cpp')
    binary.parent.mkdir(parents=True,exist_ok=True)
    if not binary.exists() or binary.stat().st_mtime < source.stat().st_mtime:
        includes=['-I'+str(p) for p in Path('/opt/ros/humble/include').iterdir() if p.is_dir()]
        subprocess.run(['g++','-std=c++17',str(source),*includes,
            '-L/opt/ros/humble/lib','-Wl,-rpath,/opt/ros/humble/lib',
            '-lrclcpp','-ltracetools','-lrosidl_runtime_c','-llibstatistics_collector','-lstatistics_msgs__rosidl_typesupport_cpp','-lrcutils','-lrmw','-lrcl','-ltf2_msgs__rosidl_typesupport_cpp',
            '-o',str(binary)],check=True)
    captured=subprocess.check_output([str(binary)],text=True,timeout=15)
    (out/'tf-authorities.tsv').write_text(captured)
    authorities={};edges={}
    for row in captured.splitlines():
        fields=row.split('\t')
        if fields[0]=='PUB':authorities[fields[1]]={'namespace':fields[2],'node':fields[3]}
        elif fields[0]=='TF':edges.setdefault(fields[1]+'->'+fields[2],set()).add(fields[3])
    if not edges:raise RuntimeError('No actual TF samples')
    clocks={}
    for name,namespace in sorted(set(node.get_node_names_and_namespaces())):
        if namespace != ns or name.startswith('transform_listener'): continue
        client=node.create_client(GetParameters,f'{namespace}/{name}/get_parameters')
        if not client.wait_for_service(timeout_sec=0.2):continue
        request=GetParameters.Request();request.names=['use_sim_time']
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=2)
        if future.done() and future.result() and future.result().values:
            clocks[f'{namespace}/{name}']=future.result().values[0].bool_value
        node.destroy_client(client)
    report={'tf_authorities':{edge:[authorities.get(gid,{'gid':gid}) for gid in sorted(gids)]
        for edge,gids in sorted(edges.items())},'use_sim_time':clocks,
        'topic_interfaces':node.get_topic_names_and_types(),
        'services':node.get_service_names_and_types()}
    (out/'live-interfaces.json').write_text(json.dumps(report,indent=2)+'\n')
    duplicates={edge:gids for edge,gids in edges.items() if len(gids)>1}
    if duplicates:raise RuntimeError(f'Multiple TF publishers for edges: {duplicates}')
    required=['point_lio','small_gicp_relocalization','controller_server','planner_server',
              'bt_navigator','loam_interface','sensor_scan_generation','fake_vel_transform',
              'terrain_analysis','terrain_analysis_ext','velocity_smoother','map_server']
    if ns+'/rviz' in clocks:required.append('rviz')
    if ns+'/joy_node' in clocks:required.append('joy_node')
    if not all(clocks.get(ns+'/'+name) is True for name in required):
        raise RuntimeError(f'Navigation clock configuration mismatch: {clocks}')
    print('Single TF authority per edge and navigation simulation clock PASS')
finally:
    node.destroy_node();rclpy.shutdown()
