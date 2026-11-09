"""Static calibrated chassis/sensor frames shared by BIEVR and TARE launches."""
from pathlib import Path

import yaml
from launch_ros.actions import Node


def load_mount(path):
    config = yaml.safe_load(Path(path).read_text())
    if not any(t['parent_frame'] == 'body' and t['child_frame'] == 'base_link'
               for t in config['transforms']):
        raise ValueError('Mount configuration needs body -> base_link')
    return config


def mount_nodes(config, use_sim_time, condition=None):
    nodes = []
    for transform in config['transforms']:
        args = []
        for key, value in zip(('x', 'y', 'z'), transform['translation']):
            args += ['--'+key, str(value)]
        for key, value in zip(('roll', 'pitch', 'yaw'), transform['rotation']):
            args += ['--'+key, str(value)]
        args += ['--frame-id', transform['parent_frame'], '--child-frame-id', transform['child_frame']]
        options = {'condition': condition} if condition is not None else {}
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name='static_tf_'+transform['parent_frame']+'_to_'+transform['child_frame'],
                          arguments=args, parameters=[{'use_sim_time': use_sim_time}], **options))
    return nodes
