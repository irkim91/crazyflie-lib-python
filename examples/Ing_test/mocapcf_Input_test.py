# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2023 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Example of how to connect to a motion capture system and feed the position to a
Crazyflie, using the motioncapture library. The motioncapture library supports all major mocap systems and provides
a generalized API regardless of system type.
The script uses the high level commander to upload a trajectory to fly a figure 8.

Set the uri to the radio settings of the Crazyflie and modify the
mocap setting matching your system.
"""
import logging
import time
from threading import Thread

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

logging.basicConfig(level=logging.ERROR)

# The host name or ip address of the mocap system
host_name = '192.168.0.20'

# The type of the mocap system
mocap_system_type = 'optitrack'

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'CF'

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

            if (max_x - min_x) < threshold and (max_y - min_y) < threshold and (max_z - min_z) < threshold:
                break

def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Flapper X, Y, Z position.
    This is going to be forwarded to the Flapper's position estimator.
    """
    #if send_full_pose:
        #cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
        #cf.extpos.send_extpose(x, y, z)

    #else:
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

def log_pos_callback(timestamp, data, logconf):
    global position_estimate
    position_estimate[0] = round(data['stateEstimate.x'], 2)
    position_estimate[1] = round(data['stateEstimate.y'], 2)
    position_estimate[2] = round(data['stateEstimate.z'], 2)
    print(f"{position_estimate[0]:.2f}, {position_estimate[1]:.2f}, {position_estimate[2]:.2f}")

def setup_logging(scf):
    logconf = LogConfig(name='Position', period_in_ms=10)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_pos_callback)
    logconf.start()


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

    roll = 0
    pitch = 15
    yawrate = 0
    zdistance = 1
    z_cmd = 0.2
    x = 0
    y = 0
    yaw = 0

    hi_cm = cf.high_level_commander
    cm = cf.commander

    # 1. Take off to position 0,0,1
    for _ in range(200):
        cm.send_position_setpoint(0, 0, zdistance, 0)
        if check_position_and_act(cm): return
        time.sleep(0.01)

    # 2. Maintain position 0,0,1
    for _ in range(200):
        cm.send_position_setpoint(0, 0, zdistance, 0)
        if check_position_and_act(cm): return
        time.sleep(0.01)

    # 3. Maintain attitude [roll, pitch, yawrate, z] = [0, 0, 0, 1]
    for _ in range(100):
        cm.send_zdistance_setpoint(0, 0, 0, zdistance)
        if check_position_and_act(cm): return
        time.sleep(0.01)

    # 4. Apply attitude [roll, pitch, yawrate, z] = [0, 10, 0, 1]
    for _ in range(50):
        cm.send_zdistance_setpoint(roll, pitch, yawrate, zdistance)
        if check_position_and_act(cm): return
        time.sleep(0.01)

    # 5. Apply attitude [roll, pitch, yawrate, z] = [0, 10, 0, 1]
    for _ in range(50):
        cm.send_zdistance_setpoint(roll, -pitch, yawrate, zdistance)
        if check_position_and_act(cm): return
        time.sleep(0.01)
        
    # 6. Maintain attitude [roll, pitch, yawrate, z] = [0, 0, 0, 1]
    for _ in range(300):
        cm.send_zdistance_setpoint(0, 0, 0, zdistance)
        if check_position_and_act(cm): return
        time.sleep(0.01)

    # 7. Move to position 0,0,1
    for _ in range(300):
        cm.send_position_setpoint(0, 0, zdistance, 0)
        if check_position_and_act(cm): return
        time.sleep(0.01)

    # 8. Land at position 0,0,0
    for _ in range(50):
        cm.send_position_setpoint(0, 0, 0, 0)
        time.sleep(0.01)    

    cf.param.set_value('usd.logging', '0')
    hi_cm.stop()
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
            activate_PID_controller(cf)
            reset_estimator(cf)
            # set_PID_gain(cf)

            logconf = setup_logging(scf)

            cf.param.set_value('usd.logging', '1')
            time.sleep(1)
            run_sequence(cf)
            cf.param.set_value('usd.logging', '0')
            logconf.stop()
            time.sleep(1)

    except Exception as e:
        print("An error occurred: {e}")
    finally:
        mocap_wrapper.close()