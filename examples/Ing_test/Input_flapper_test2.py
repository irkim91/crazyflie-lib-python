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

import numpy as np

import motioncapture

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# The host name or ip address of the mocap system
host_name = '192.168.0.20'

# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'optitrack'

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'Flapper2'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3
BOX_LIMIT = 1.5
position_estimate = [0, 0, 0]

class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    if self.on_pose:
                        pos = obj.position
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])

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

def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Flapper X, Y, Z position.
    This is going to be forwarded to the Flapper's position estimator.
    """
    cf.extpos.send_extpos(x, y, z)

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(cf)

def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)

def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

def activate_PID_controller(cf):
    cf.param.set_value('stabilizer.controller', '1')
    print('PID controller selected')

def set_Flapper(cf):

    # Thrust value
    cf.param.set_value('posCtlPid.thrustBase','32000')
    #cf.param.set_value('posCtlPid.thrustBase','0')

    cf.param.set_value('flapper.servPitchNeutr','50')
    cf.param.set_value('flapper.servYawNeutr','50')
    cf.param.set_value('flapper.flapperMaxThrust','50000')
    #cf.param.set_value('flapper.rollCut','20')
    #cf.param.set_value('flapper.pitchCut','20')
    #cf.param.set_value('flapper.yawCut','5.0')
    
    print('Flapper is set up')

def set_PID_gain(cf):
    
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


def move_to_center_and_land(cm):
    # move to center
    for _ in range(500):
        cm.send_position_setpoint(0, 0, 1, 0)
        time.sleep(0.01)

    # landing
    for _ in range(50):
        cm.send_position_setpoint(0, 0, 0, 0)
        time.sleep(0.01) 

def check_position_and_act(cm):
    if abs(position_estimate[0]) > BOX_LIMIT or abs(position_estimate[1]) > BOX_LIMIT:
        print("Warning: Position limit exceeded. Moving to center and landing.")
        move_to_center_and_land(cm)
        return True
    return False


def run_sequence(cf):
    print('run sequence')

    thrust  = 33000
    roll    = 0
    pitch   = -5800
    yaw     = 0

    roll_cmd    = 0
    pitch_cmd   = 3000
    yaw_cmd     = 0

    vx = 0
    vy = 0
    yawrate     = 0
    zdistance   = 1

    # 진폭 설정
    amplitude = 10000

    # 시작 주파수와 끝 주파수 (Hz)
    start_freq = 0
    end_freq = 5

    # 주파수 변화 범위
    freq_range = np.arange(start_freq, end_freq + 0.5, 0.5)


    hi_cm = cf.high_level_commander
    cm = cf.commander
    
    # 1. Take off to position 0,0,1
    for _ in range(200):
        cm.send_position_setpoint(0, 0, zdistance, 0)
        if check_position_and_act(cm): return
        time.sleep(0.01)
    
    # 2. Maintain position 0,0,1
    for _ in range(500):
        cm.send_position_setpoint(0, 0, zdistance, 0)
        #cm.send_hover_setpoint(0,0,0, zdistance)
        if check_position_and_act(cm): return
        time.sleep(0.01)


    # 
    for _ in range(50):
        cm.send_longmanual_control(thrust, pitch, roll, yawrate)
        time.sleep(0.01)

    for _ in range(50):
        cm.send_longmanual_control(thrust, pitch+pitch_cmd, roll, yawrate)
        time.sleep(0.01)

    for _ in range(50):
        cm.send_longmanual_control(thrust, pitch-pitch_cmd, roll, yawrate)
        time.sleep(0.01)

    '''
    for freq in freq_range:
        # 주기 계산 (T = 1/freq), freq가 0인 경우 skip
        if freq == 0:
            continue  # 0Hz에서는 사인파를 만들 수 없으므로 생략
    
        period = 1 / freq
    
        # 1Hz 당 5번의 주기를 가지기 위해 5 * period만큼 반복
        num_cycles = 5
        total_time_for_freq = num_cycles * period  # 주파수 당 전체 시간
    
        # 주파수 당 몇 번의 반복을 할지 계산 (0.01초마다 명령을 보내므로)
        num_iterations = int(total_time_for_freq / 0.01)
    
        for i in range(num_iterations):
            # 현재 시간 계산
            current_time = i * 0.01
        
            # 사인파 생성 (진폭 1000, 주파수 고정)
            pitch = amplitude * np.sin(2 * np.pi * freq * current_time)
        
        
            # 명령 전송
            cm.send_longmanual_control(thrust, pitch, roll, yawrate)
        
            # 10ms 대기
            time.sleep(0.01) 
    '''
    '''
    for _ in range(50):
        cm.send_longmanual_control(thrust, pitch+pitch_cmd, roll, yawrate)
        time.sleep(0.01)

    for _ in range(50):
        cm.send_longmanual_control(thrust, pitch-pitch_cmd, roll, yawrate)
        time.sleep(0.01)

    for _ in range(500):
        cm.send_longmanual_control(thrust, pitch, roll, yawrate)
        time.sleep(0.01)
    '''
    
    for _ in range(500):
        cm.send_position_setpoint(0, 0, zdistance, 0)
        time.sleep(0.01)
    
    
    # 8. Land at position 0,0,0
    for _ in range(200):
        cm.send_position_setpoint(0, 0, 0, 0)
        time.sleep(0.01)
    
    for _ in range(100):
        cm.send_stop_setpoint()
        time.sleep(0.01)

    time.sleep(1)

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Connect to the mocap system
    mocap_wrapper = MocapWrapper(rigid_body_name)

    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            # Set up a callback to handle data from the mocap system
            mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])
            print('Connecting to Motion capture')
            
            adjust_orientation_sensitivity(cf)
            activate_kalman_estimator(cf)
            #activate_complementary_estimator(cf)
            activate_PID_controller(cf)
            set_Flapper(cf)
            set_PID_gain(cf)
            reset_estimator(cf)

            cf.param.set_value('usd.logging', '1')
            time.sleep(1)
            run_sequence(cf)
            cf.param.set_value('usd.logging', '0')
            time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        mocap_wrapper.close()
