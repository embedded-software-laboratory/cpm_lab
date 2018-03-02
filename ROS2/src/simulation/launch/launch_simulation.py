#!/usr/bin/python3

import os
import sys

from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher
from ros2run.api import get_executable_path


def main():
    launcher = DefaultLauncher()
    launch_descriptor = LaunchDescriptor()
    package = 'simulation'

    launch_descriptor.add_process(cmd=[get_executable_path(package_name=package, executable_name='simple_simulation')])
    launch_descriptor.add_process(cmd=[get_executable_path(package_name=package, executable_name='simple_controller')])

    launcher.add_launch_descriptor(launch_descriptor)

    rc = launcher.launch()
    if rc != 0:
        print('Something went wrong. Return code: ' + str(rc), file=sys.stderr)
        exit(rc)


if __name__ == '__main__':
    main()
