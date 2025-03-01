#!/usr/bin/env python3

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

# ROS 2 module imports
from std_msgs.msg import Int32, Float32, Bool # Int32, Float32 and Bool message classes
from geometry_msgs.msg import Point # Point message class
from sensor_msgs.msg import JointState, Imu, LaserScan, Image # JointState, Imu, LaserScan and Image message classes

# Python mudule imports
try: # Mapping objects that allow their elements to be accessed both as keys and as attributes
    from attrdict import AttrDict
except ImportError: # attrdict broken in Python 3.10 and not maintained
    # Monkey patch collections
    import collections
    import collections.abc
    for type_name in collections.abc.__all__:
        setattr(collections, type_name, getattr(collections.abc, type_name))
    from attrdict import AttrDict

################################################################################

# ROS 2 publishers and subscribers
pub_sub_dict = AttrDict({
    'subscribers': [
        {'topic':'/autodrive/f1tenth_1/throttle_command', 'type': Float32, 'name': 'sub_throttle_command'},
        {'topic':'/autodrive/f1tenth_1/steering_command', 'type': Float32, 'name': 'sub_steering_command'},
        {'topic':'/autodrive/reset_command', 'type': Bool, 'name': 'callback_reset_command'}
    ],
    'publishers': [
        {'topic': '/autodrive/f1tenth_1/throttle', 'type': Float32, 'name': 'pub_throttle'},
        {'topic': '/autodrive/f1tenth_1/steering', 'type': Float32, 'name': 'pub_steering'},
        {'topic': '/autodrive/f1tenth_1/speed', 'type': Float32, 'name': 'pub_speed'},
        {'topic': '/autodrive/f1tenth_1/left_encoder', 'type': JointState, 'name': 'pub_left_encoder'},
        {'topic': '/autodrive/f1tenth_1/right_encoder', 'type': JointState, 'name': 'pub_right_encoder'},
        {'topic': '/autodrive/f1tenth_1/ips', 'type': Point, 'name': 'pub_ips'},
        {'topic': '/autodrive/f1tenth_1/imu', 'type': Imu, 'name': 'pub_imu'},
        {'topic': '/autodrive/f1tenth_1/lidar', 'type': LaserScan, 'name': 'pub_lidar'},
        {'topic': '/autodrive/f1tenth_1/front_camera', 'type': Image, 'name': 'pub_front_camera'},
        {'topic': '/autodrive/f1tenth_1/lap_count', 'type': Int32, 'name': 'pub_lap_count'},
        {'topic': '/autodrive/f1tenth_1/lap_time', 'type': Float32, 'name': 'pub_lap_time'},
        {'topic': '/autodrive/f1tenth_1/last_lap_time', 'type': Float32, 'name': 'pub_last_lap_time'},
        {'topic': '/autodrive/f1tenth_1/best_lap_time', 'type': Float32, 'name': 'pub_best_lap_time'},
        {'topic': '/autodrive/f1tenth_1/collision_count', 'type': Int32, 'name': 'pub_collision_count'}
    ]
})