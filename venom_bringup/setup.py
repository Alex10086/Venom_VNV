from glob import glob
import os
from typing import List, Sequence, Tuple

from setuptools import find_packages, setup


package_name = 'venom_bringup'
package_root = os.path.dirname(os.path.realpath(__file__))
setup_cwd = os.getcwd()

DataFile = Tuple[str, Sequence[str]]


def package_path(*parts: str) -> str:
    return os.path.join(package_root, *parts)


def path_for_setup(path: str) -> str:
    return os.path.relpath(path, setup_cwd)


def required_package_files(*relative_paths: str) -> List[str]:
    paths = [package_path(path) for path in relative_paths]
    missing = [path for path in paths if not os.path.isfile(path)]
    if missing:
        raise RuntimeError('Missing package data files: ' + ', '.join(missing))
    return [path_for_setup(path) for path in paths]


def collect_flat_data_files(relative_dir: str, patterns: Sequence[str]) -> List[DataFile]:
    source_dir = package_path(relative_dir)
    files: List[str] = []
    for pattern in patterns:
        files.extend(sorted(glob(os.path.join(source_dir, pattern))))
    if not files:
        return []
    install_dir = os.path.join('share', package_name, relative_dir)
    return [(install_dir, [path_for_setup(path) for path in files])]


def collect_config_data_files() -> List[DataFile]:
    config_root = package_path('config')
    data_files: List[DataFile] = []
    for root, _, _ in os.walk(config_root):
        yaml_files = sorted(glob(os.path.join(root, '*.yaml')))
        json_files = sorted(glob(os.path.join(root, '*.json')))
        files = [path_for_setup(path) for path in yaml_files + json_files]
        if not files:
            continue
        rel_root = os.path.relpath(root, config_root)
        install_dir = os.path.join('share', package_name, 'config')
        if rel_root != '.':
            install_dir = os.path.join(install_dir, rel_root)
        data_files.append((install_dir, files))
    return data_files


def collect_launch_data_files() -> List[DataFile]:
    launch_root = package_path('launch')
    data_files: List[DataFile] = []
    for root, _, _ in os.walk(launch_root):
        py_files = [path_for_setup(path) for path in sorted(glob(os.path.join(root, '*.py')))]
        if not py_files:
            continue
        rel_root = os.path.relpath(root, launch_root)
        install_dir = os.path.join('share', package_name, 'launch')
        if rel_root != '.':
            install_dir = os.path.join(install_dir, rel_root)
        data_files.append((install_dir, py_files))
    return data_files


def collect_map_data_files() -> List[DataFile]:
    return collect_flat_data_files('map', ('*.yaml', '*.pgm'))


data_files: List[DataFile] = [
    ('share/ament_index/resource_index/packages',
        required_package_files('resource/' + package_name)),
    ('share/' + package_name, required_package_files('package.xml')),
]
data_files += collect_flat_data_files('rviz_cfg', ('*.rviz',))
data_files += collect_flat_data_files('scripts', ('*.sh',))
data_files += collect_launch_data_files()
data_files += collect_config_data_files()
data_files += collect_map_data_files()


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='venom',
    maintainer_email='liyihan.xyz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'multi_waypoint_commander = '
            'venom_bringup.multi_waypoint_commander:main',
            'odom_path_publisher = '
            'venom_bringup.odom_path_publisher:main',
        ],
    },
)
