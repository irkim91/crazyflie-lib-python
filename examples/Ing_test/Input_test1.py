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

    # Thrust value
    cf.param.set_value('posCtlPid.thrustBase','32000')
    
    # X axis gain
    cf.param.set_value('posCtlPid.xKp', '1')      # default : 1.5
    cf.param.set_value('velCtlPid.vxKp', '10')      # default : 25
    cf.param.set_value('velCtlPid.vxKi', '0.5')     # default : 1
    cf.param.set_value('velCtlPid.vxKd', '0')       # default : 0
    cf.param.set_value('velCtlPid.vxKFF', '10')     # default : 10

    # Y axis gain
    cf.param.set_value('posCtlPid.yKp', '1')      # default : 1.5
    cf.param.set_value('velCtlPid.vyKp', '10')      # default : 15
    cf.param.set_value('velCtlPid.vyKi', '0.5')       # default : 1
    cf.param.set_value('velCtlPid.vyKd', '0')       # default : 0
    cf.param.set_value('velCtlPid.vyKFF', '5')      # default : 5

    # Z axis gain
    cf.param.set_value('posCtlPid.zKp', '5')        # default : 5
    cf.param.set_value('posCtlPid.zKi', '0.5')      # default : 0.5
    cf.param.set_value('velCtlPid.vzKp', '12.5')    # default : 12.5
    cf.param.set_value('velCtlPid.vzKi', '0.5')     # default : 0.5
    
    # roll gain
    cf.param.set_value('pid_attitude.roll_kp', '10')  #14
    cf.param.set_value('pid_attitude.roll_kd', '0')
    cf.param.set_value('pid_rate.roll_kp', '50')   #50
    cf.param.set_value('pid_rate.roll_kd', '0')   #0.5
    cf.param.set_value('pid_rate.omxFiltCut', '12.5')
    
    # pitch gain
    cf.param.set_value('pid_attitude.pitch_kp', '30') #14
    cf.param.set_value('pid_attitude.pitch_kd', '0')
    cf.param.set_value('pid_rate.pitch_kp',   '50')   #110
    cf.param.set_value('pid_rate.pitch_ki',   '0')     #0
    cf.param.set_value('pid_rate.pitch_kd',   '0')     #2
    cf.param.set_value('pid_rate.omyFiltCut', '12.5')

    # yaw gain
    cf.param.set_value('pid_attitude.yaw_kp', '30')
    cf.param.set_value('pid_attitude.yaw_kd', '0')
    cf.param.set_value('pid_rate.yaw_kp', '30')
    cf.param.set_value('pid_rate.yaw_kff', '220')
    cf.param.set_value('pid_rate.omzFiltCut', '3')
    
    
    print('PID gain is set up')

def run_sequence(cf):
    print('run sequence')

    roll = 0
    pitch = 5
    yawrate = 0
    zdistance = 1
    z_cmd = 0.2
    x = 0
    y = 0
    yaw = 0

    vx = 0
    vy = 0

    hi_cm = cf.high_level_commander
    cm = cf.commander

    # 1. Take off to attitude [roll, pitch, yawrate, z] = [0, 0, 0, 1]
    for _ in range(200):
        cm.send_hover_setpoint(vx, vy, yawrate, zdistance)
        time.sleep(0.01)
    '''
    # 2. Maintain attitude [roll, pitch, yawrate, z] = [0, 0, 0, 1]
    for _ in range(200):
        cm.send_zdistance_setpoint(0, 0, 0, zdistance)
        time.sleep(0.01)

    # 4. Apply attitude [roll, pitch, yawrate, z] = [0, 10, 0, 1]
    for _ in range(50):
        cm.send_zdistance_setpoint(roll, pitch, yawrate, zdistance)
        time.sleep(0.01)

    # 5. Apply attitude [roll, pitch, yawrate, z] = [0, 10, 0, 1]
    for _ in range(50):
        cm.send_zdistance_setpoint(roll, -pitch, yawrate, zdistance)
        time.sleep(0.01)

    # 4. Apply attitude [roll, pitch, yawrate, z] = [0, 10, 0, 1]
    for _ in range(50):
        cm.send_zdistance_setpoint(roll, pitch, yawrate, zdistance)
        time.sleep(0.01)

    # 5. Apply attitude [roll, pitch, yawrate, z] = [0, 10, 0, 1]
    for _ in range(50):
        cm.send_zdistance_setpoint(roll, -pitch, yawrate, zdistance)
        time.sleep(0.01)
    '''

    for i in range(50):
        cf.param.set_value('motorPowerSet.m1',  4815)
        cf.param.set_value('motorPowerSet.m3', -8347)
        cf.param.set_value('motorPowerSet.m2',  50 + 32684)
        cf.param.set_value('motorPowerSet.m4', -50 + 32684)
        #cf.param.set_value('motorPowerSet.m4', 0)
        time.sleep(0.01)


    # 1. Take off to attitude [roll, pitch, yawrate, z] = [0, 0, 0, 1]
    for _ in range(200):
        cm.send_hover_setpoint(vx, vy, yawrate, zdistance)
        time.sleep(0.01)

    # 8. Land at position 0,0,0
    for _ in range(50):
        cm.send_hover_setpoint(vx, vy, yawrate, 0)
        time.sleep(0.01)    

    hi_cm.stop()
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