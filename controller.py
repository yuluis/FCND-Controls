"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM
import time as timer

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0

class NonlinearController(object):



    def __init__(self):
        """Initialize the controller object and control gains"""

        self.g = 9.81
        self.rot_mat = np.eye(3)
        self.start_time = timer.time()

        return

    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory
        
        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds
            
        Returns: tuple (commanded position, commanded velocity, commanded yaw)
                
        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]
        
        
        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]
            
            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]
            
        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]
                
                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]
            
        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)
        

        return (position_cmd, velocity_cmd, yaw_cmd)
    
    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command
            
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        return np.array([0.0, 0.0])
    
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            acceleration_ff: feedforward acceleration command (+up)

        """
        mot_mat = euler2RM(attitude[0], attitude[1], attitude[2])
        b_z = self.rot_mat[2,2]
        z_k_p = 1
        z_k_d = 1

        z_err = altitude_cmd - altitude
        z_err_dot = vertical_velocity_cmd - vertical_velocity
        p_term = z_k_p * z_err
        d_term = z_k_d * z_err_dot

        u_1_bar = p_term + d_term + acceleration_ff #PD controller
        c = (u_1_bar + self.g) / b_z #factor in self frame relative to Euler frame, self.g is accel needed to zero out gravity

        thrust = DRONE_MASS_KG * c #thrust is in the body frame?

        #print("altitude_control:: time= {0:.4f}, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, accel, thrust)".format(timer.time()),
        #     altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff, thrust)

        return thrust  #    Returns: thrust command for the vehicle (+up)




    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll,pitch,yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        k_p_rollpitch = 1

        self.rot_mat = euler2RM (acceleration_cmd[0], acceleration_cmd[1],thrust_cmd)
        b_x = self.rot_mat[0, 2]
        b_x_err = b_x - attitude[0]
        b_x_p_term = k_p_rollpitch * b_x_err

        b_y = self.rot_mat[1, 2]
        b_y_err = b_y - attitude[1]
        b_y_p_term = k_p_rollpitch * b_y_err

        b_x_commanded_dot = b_x_p_term
        b_y_commanded_dot = b_y_p_term

        rot_mat1=np.array([[self.rot_mat[1,0],-self.rot_mat[0,0]],[self.rot_mat[1,1],-self.rot_mat[0,1]]])/self.rot_mat[2,2]

        rot_rate = np.matmul(rot_mat1,np.array([b_x_commanded_dot,b_y_commanded_dot]).T)
        tau=2*DRONE_MASS_KG* 1
        p_c = rot_rate[0]/tau * MOI[0] # TODO factor in rotational moment? are p and q body frame moments?
        q_c = rot_rate[1]/tau * MOI[1]


        #print("roll_pitch_controller:: time= {0:.4f}, accel cmd, attitude, thrust_cmd, p_c, q_c)".format(timer.time()),
        #      acceleration_cmd, attitude, thrust_cmd, np.array([p_c, q_c]) )
        if attitude[0] > 1 :
            print("attitude", attitude)
            pass

        return np.array([p_c, q_c])

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:R
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            attitude: 3-element numpy array (p,q,r) in radians/second^2
        """
        body_p = 1

        u_bar_p = (body_rate_cmd[0] - body_rate[0]) * body_p * MOI[0]

        u_bar_q = (body_rate_cmd[1] - body_rate[1]) * body_p * MOI[1]

        u_bar_r = (body_rate_cmd[2] - body_rate[2]) * body_p * MOI[2]

        body_new = np.array([u_bar_p, u_bar_q, u_bar_r])

        #print("body_rate_control:: time= {0:.4f}, body_rate (cmd, curr, new)".format(timer.time()), body_rate_cmd, body_rate, body_new )

        #Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        return body_new



    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrateD
        
        Args:
            yaw_cmd: desired vehicle yaw in Dradians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/secRR
        """
        yaw_p = 1
        psi_err = (yaw_cmd - yaw)
        r_c = psi_err * yaw_p

        print("yaw_control::time= {0:.4f}, yaw (cmd, curr, new)".format(timer.time()), yaw_cmd, yaw, r_c )

        return r_c

