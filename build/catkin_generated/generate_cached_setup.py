# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
<<<<<<< HEAD
    for workspace in '/home/diego/scara_ros/devel;/opt/ros/noetic'.split(';'):
=======
    for workspace in '/opt/ros/noetic'.split(';'):
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

<<<<<<< HEAD
code = generate_environment_script('/home/diego/scara_ros/devel/env.sh')

output_filename = '/home/diego/scara_ros/build/catkin_generated/setup_cached.sh'
=======
code = generate_environment_script('/home/rabios/dev/scara_ros/devel/env.sh')

output_filename = '/home/rabios/dev/scara_ros/build/catkin_generated/setup_cached.sh'
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
