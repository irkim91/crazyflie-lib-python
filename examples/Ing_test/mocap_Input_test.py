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
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

logging.basicConfig(level=logging.ERROR)

# The host name or ip address of the mocap system
host_name = '192.168.0.20'

# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'optitrack'

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'F1'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3

BOX_LIMIT = 1.5
position_estimate = [0, 0]
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
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if send_full_pose:
        #cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
        cf.extpos.send_extpos(x, y, z)

    else:
        cf.extpos.send_extpos(x, y, z)


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    # time.sleep(1)
    wait_for_position_estimator(cf)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)


def activate_PID_controller(cf):
    cf.param.set_value('stabilizer.controller', '1')


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

def move_box_limit(cf, mocap_wrapper):

    cm = cf.commander

    body_x_cmd = 0.2
    body_y_cmd = 0.1
    max_vel = 0.2

    while (1):

        if position_estimate[0] > BOX_LIMIT:
            body_x_cmd = -max_vel
        elif position_estimate[0] < -BOX_LIMIT:
            body_x_cmd = max_vel
        if position_estimate[1] > BOX_LIMIT:
            body_y_cmd = -max_vel
        elif position_estimate[1] < -BOX_LIMIT:
            body_y_cmd = max_vel

        cm.send_hover_setpoint(body_x_cmd, body_y_cmd, 0, 1)
        time.sleep(0.1)

def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']

def run_sequence(cf):

    print('run sequence')
    pitch_neutral = 63
    yaw_neutral = 62

    cf.param.set_value('flapper.motBiasRoll', 0)  # % unit
    cf.param.set_value('flapper.servPitchNeutr', pitch_neutral)  # % unit
    cf.param.set_value('flapper.servYawNeutr', yaw_neutral)  # % unit
    cf.param.set_value('flapper.flapperMaxThrust',50000)

    TT      = 10

    x = 0
    y = 0
    zdistance = 1
    yaw = 0

    vx  = 0
    vy  = 0

    pitch   = 10
    roll    = 0
    yawrate = 0

    dt = 1    # hi_cm.takeoff(zdistance, 2.0)
    relative = True

    hi_cm = cf.high_level_commander
    cm = cf.commander

    # rest command
    for i in range(200):
        #cm.send_position_setpoint( 0, 0, zdistance, 0)
        cm.send_hover_setpoint(0, 0, 0, zdistance)
        #cm.send_zdistance_setpoint(0, -1.1, 0, zdistance)
        time.sleep(0.01)
    
    # doublet command
    '''
    for i in range(TT):
        cm.send_hover_setpoint(vx, vy, yawrate, zdistance)
        time.sleep(0.01)
    
    # rest
    for i in range(200):
        cm.send_position_setpoint( 0, y, zdistance, yaw)
        time.sleep(0.01)
    '''
    # sequence
    for i in range(500):
        if abs(position_estimate[0]) > BOX_LIMIT or abs(position_estimate[1]) > BOX_LIMIT:
            cm.send_hover_setpoint(0, 0, 0, zdistance)
            time.sleep(0.1)
            break
        cm.send_zdistance_setpoint(0, pitch, 0, zdistance)
        time.sleep(0.01)
    '''
    for i in range(30):
        if abs(position_estimate[0]) > BOX_LIMIT or abs(position_estimate[1]) > BOX_LIMIT:
            cm.send_hover_setpoint(0, 0, 0, zdistance)
            time.sleep(0.1)
            break
        cm.send_zdistance_setpoint(roll, pitch, yawrate, zdistance)
        time.sleep(0.01)

    for i in range(20):
        if abs(position_estimate[0]) > BOX_LIMIT or abs(position_estimate[1]) > BOX_LIMIT:
            cm.send_hover_setpoint(0, 0, 0, zdistance)
            time.sleep(0.1)
            break
        cm.send_zdistance_setpoint(roll, -pitch, yawrate, zdistance)
        time.sleep(0.01)
    
    for i in range(100):
        if abs(position_estimate[0]) > BOX_LIMIT or abs(position_estimate[1]) > BOX_LIMIT:
            cm.send_hover_setpoint(0, 0, 0, zdistance)
            time.sleep(0.1)
            break
        cm.send_zdistance_setpoint(roll, pitch, yawrate, zdistance)
        time.sleep(0.01)

    for i in range(100):
        if abs(position_estimate[0]) > BOX_LIMIT or abs(position_estimate[1]) > BOX_LIMIT:
            cm.send_hover_setpoint(0, 0, 0, zdistance)
            time.sleep(0.1)
            break
        cm.send_zdistance_setpoint(roll, -pitch, yawrate, zdistance)
        time.sleep(0.01)
    '''    
    for i in range(500):
        if abs(position_estimate[0]) > BOX_LIMIT or abs(position_estimate[1]) > BOX_LIMIT:
            cm.send_hover_setpoint(0, 0, 0, zdistance)
            time.sleep(0.1)
            break
        cm.send_zdistance_setpoint(0, -pitch, 0, zdistance)
        time.sleep(0.01)
    
    # rest
    for i in range(500):
        cm.send_position_setpoint( 0, 0, zdistance, 0)
        time.sleep(0.01)
    
    # ladning
    for i in range(50):
        cm.send_position_setpoint( 0, 0, 0, 0)
        #cm.send_hover_setpoint(0, 0, 0, 0)
        time.sleep(0.01)

    hi_cm.stop()
    time.sleep(1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Connect to the mocap system
    mocap_wrapper = MocapWrapper(rigid_body_name)

    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf
            trajectory_id = 1

            # Set up a callback to handle data from the mocap system
            mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])
            print('Connecting to Motion capture')
            
            adjust_orientation_sensitivity(cf)
            activate_kalman_estimator(cf)
            activate_PID_controller(cf)
            reset_estimator(cf)
            set_PID_gain(cf)

            logconf = LogConfig(name='Position', period_in_ms=10)
            logconf.add_variable('stateEstimate.x', 'float')
            logconf.add_variable('stateEstimate.y', 'float')
            scf.cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(log_pos_callback)
            logconf.start()

            cf.param.set_value('usd.logging', '1')
            time.sleep(1)
            run_sequence(cf)
            cf.param.set_value('usd.logging', '0')
            logconf.stop()
            time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        mocap_wrapper.close()