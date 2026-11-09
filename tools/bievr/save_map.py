#!/usr/bin/env python3
"""Save occupancy map plus the SLAM pose graph required for later localization."""
import argparse
import json
from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('prefix', help='Absolute map prefix without extension')
    options = parser.parse_args()
    prefix = Path(options.prefix).expanduser().resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    if any(Path(str(prefix)+suffix).exists() for suffix in ('.yaml', '.posegraph', '.data')):
        raise FileExistsError('Map already exists; choose a new prefix: '+str(prefix))
    rclpy.init()
    node = Node('save_bievr_navigation_map')
    try:
        for name, srv in [('save_map', SaveMap), ('serialize_map', SerializePoseGraph)]:
            client = node.create_client(srv, '/slam_toolbox/'+name)
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('SLAM service unavailable: '+name)
            request = srv.Request()
            if name == 'save_map':
                request.name.data = str(prefix)
            else:
                request.filename = str(prefix)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=20.)
            if not future.done() or future.result() is None or future.result().result != 0:
                raise RuntimeError('SLAM map save failed: '+name)
        for suffix in ('.yaml', '.posegraph', '.data'):
            if not Path(str(prefix)+suffix).is_file():
                raise RuntimeError('Missing saved artifact: '+str(prefix)+suffix)
        mount = Path(get_package_share_directory('venom_bringup'))/'config/scout_mini/bievr/mount.yaml'
        shutil.copy2(mount, str(prefix)+'.mount.yaml')
        print(json.dumps({'saved': str(prefix), 'localization_command':
                         'bash tools/bievr/start_bievr.sh localization map_file:='+str(prefix)}, indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
