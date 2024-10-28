# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the first Crazyflie found, ramps up/down
the motors and disconnects.
"""
import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def activate_complementary_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '1')
    print('complementary estimator selected')

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    # time.sleep(1)
    wait_for_position_estimator(cf)

def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

def activate_PID_controller(cf):
    cf.param.set_value('stabilizer.controller', '1')
    print('PID controller selected')

def set_PID_gain(cf):
    gains = {
        'posCtlPid.thrustBase': 32000,
        # X pos,vel
        'posCtlPid.xKp'     : 1,
        'velCtlPid.vxKp'    : 10,
        'velCtlPid.vxKi'    : 0.5,
        'velCtlPid.vxKd'    : 0,
        'velCtlPid.vxKFF'   : 10,
        # Y pos,vel
        'posCtlPid.yKp'     : 1,
        'velCtlPid.vyKp'    : 10,
        'velCtlPid.vyKi'    : 0.5,
        'velCtlPid.vyKd'    : 0,
        'velCtlPid.vyKFF'   : 5,
        # Z pos,vel
        'posCtlPid.zKp'     : 5,
        'posCtlPid.zKi'     : 0.5,
        'velCtlPid.vzKp'    : 12.5,
        'velCtlPid.vzKi'    : 0.5,
        # Roll ang,rate
        'pid_attitude.roll_kp'  : 10,
        'pid_attitude.roll_kd'  : 0,
        'pid_rate.roll_kp'      : 50,
        'pid_rate.roll_kd'      : 0,
        'pid_rate.omxFiltCut'   : 12.5,
        # Pitch ang,rate
        'pid_attitude.pitch_kp' : 30,
        'pid_attitude.pitch_kd' : 0,
        'pid_rate.pitch_kp'     : 50,
        'pid_rate.pitch_ki'     : 0,
        'pid_rate.pitch_kd'     : 2,
        'pid_rate.omyFiltCut'   : 12.5,
        # Yaw ang,rate
        'pid_attitude.yaw_kp'   : 30,
        'pid_attitude.yaw_kd'   : 0,
        'pid_rate.yaw_kp'       : 30,
        'pid_rate.yaw_kff'      : 220,
        'pid_rate.omzFiltCut'   : 3
    }

    for param, value in gains.items():
        cf.param.set_value(param, value)

    print('PID gain is set up')

def run_sequence(cf):
    print('run sequence')

    thrust  = 10000
    roll    = 0
    pitch   = 0
    yaw     = 0

    hi_cm = cf.high_level_commander
    cm = cf.commander

    # 1. Take off to attitude 
    for _ in range(200):
        cm.send_manual_control(thrust, roll, pitch, yaw)
        time.sleep(0.01)

    for _ in range(200):
        cm.send_manual_control(10000, 0000, 5000, 0)
        time.sleep(0.01)

    for _ in range(100):
        cm.send_manual_control(0,0,0,0)
        time.sleep(0.01)
    time.sleep(3)

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        
        activate_kalman_estimator(cf)
        #activate_complementary_estimator(cf)
        activate_PID_controller(cf)
        set_PID_gain(cf)
        reset_estimator(cf)
        cf.param.set_value('usd.logging', '1')
        time.sleep(1)
        run_sequence(cf)
        cf.param.set_value('usd.logging', '0')
        time.sleep(1)