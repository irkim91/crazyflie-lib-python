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
rigid_body_name = 'Flapper4'

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
    #cf.param.set_value('posCtlPid.thrustBase','32000')
    cf.param.set_value('posCtlPid.thrustBase','0')

    #cf.param.set_value('flapper.servPitchNeutr','45')
    #cf.param.set_value('flapper.servYawNeutr','58')
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


def battery_voltage_callback(timestamp, data, logconf):
    global vbat, thrust
    vbat = data['flapper.vbat']
    thrust = 255700 / vbat
    print(f"battery voltage : {vbat:.2f} V, Updated Thrust : {thrust:.2f}")

def setup_voltage_logger(cf):
    log_conf = LogConfig(name='Battery', period_in_ms=10)
    log_conf.add_variable('flapper.vbat','float')
    cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(battery_voltage_callback)
    log_conf.start()


def run_sequence(cf):
    print('run sequence')

    thrust2 = 1000
    roll    = 0
    pitch   = 0
    yaw     = 0

    roll_cmd    = 0
    pitch_cmd   = 0
    yaw_cmd     = 0

    vx = 0
    vy = 0
    yawrate     = 0
    zdistance   = 1

    hi_cm = cf.high_level_commander
    cm = cf.commander
    
    # 진폭 설정
    amplitude = 30000

    # 시작 주파수와 끝 주파수 (Hz)
    start_freq  = 1
    end_freq    = 6
    '''
    # 최소 주파수에 해당하는 최대 주기 계산
    max_period = 2 * np.pi / start_freq  # 주기 (T = 2π/ω)
    
    # 기록 시간은 4.5배 이상의 최대 주기로 설정 (Trec ≥ 4.5 * Tmax)
    sweep_time = 5 * max_period  # 자동으로 계산된 스윕 시간

    # 주파수 변화 범위
    freq_range = np.linspace(start_freq, end_freq, int(sweep_time / 0.01))
    
    # 각 주파수에서 1주기 시간 계산
    frequencies = np.arange(start_freq, end_freq)
    periods = 2 * np.pi / frequencies

    # 전체 sweep 시간
    sweep_time = np.sum(periods)
    
    # 각 주파수 구간에서 점진적으로 주파수 변화
    freq_range = np.linspace(start_freq, end_freq, int(sweep_time / 0.01))
    '''

    # 주파수 변화 범위 (1Hz 단위)
    frequencies = np.arange(start_freq, end_freq + 1, 1)

    '''
    t = np.linspace(0,20,1000)

    half_time = len(t) //4 

    amplitude_one_cycle = np.concatenate([
        np.linspace(0, amplitude, half_time),           # 0 -> 30000
        np.linspace(amplitude, -amplitude, half_time),  # 30000 -> -30000
        np.linspace(-amplitude, 0, half_time)           # -30000 -> 0
    ])

    amplitude_two_cycles = np.tile(amplitude_one_cycle, 1) 
    
    for i, freq in enumerate(freq_range):
        current_time = i * 0.01
        pitch = -amplitude * np.sin(freq * current_time)
        cm.send_manual_control(thrust, 0,pitch, 0)
        time.sleep(0.01)
    '''
    for freq in frequencies:
        period = 2 * np.pi / freq  # 각 주파수의 최대 주기 계산
        num_cycles = 5  # 각 주파수에서 10주기 반복
        total_time = num_cycles * period  # 각 주파수에서 반복할 총 시간
        time_steps = np.arange(0, total_time, 0.01)  # 0.01초마다 샘플링

        for current_time in time_steps:
            pitch = -amplitude * np.sin(2 * np.pi * freq * current_time)
            cm.send_manual_control(thrust, 0, pitch, 0)
            time.sleep(0.01)  # 0.01초 대기    
    '''
    for i in range(len(amplitude_two_cycles)):
        pitch = amplitude_two_cycles[i]      
        cm.send_manual_control(10000, 0, pitch, 0)
        time.sleep(0.01)  # 0.01초 대기
    '''
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

            setup_voltage_logger(cf)

            cf.param.set_value('usd.logging', '1')
            time.sleep(1)
            run_sequence(cf)
            cf.param.set_value('usd.logging', '0')
            time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        mocap_wrapper.close()
