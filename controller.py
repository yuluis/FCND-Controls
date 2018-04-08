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
#test

class NonlinearController(object):



    def __init__(self):
        """Initialize the controller object and control gains"""

        self.rot_mat = np.eye(3)
        self.start_time = timer.time()

        self.accum_yaw_err = 0  # integrator
        self.yaw_counter = 0
        self.accum_yaw_accel_update = 0  # current yaw acceleration target

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
        # try slowing down the updates to allow time for inner control loops to work
        # what is c?
        # try printing out the values
        #mot_mat = euler2RM(attitude[0], attitude[1], attitude[2])
        #b_z = self.rot_mat[2,2]
        #u_1_bar = p_term + d_term + acceleration_ff #PD controller
        #c = (u_1_bar - GRAVITY) / b_z #factor in self frame relative to Euler frame, self.g is accel needed to zero out gravity

        k_p = 4
        k_d = 0.2
        c = 1 # what is this?
        x_err = local_position_cmd[0] - local_position[0]
        x_err_dot = local_velocity_cmd[0] - local_velocity[0]

        p_term_x = k_p * x_err
        d_term_x = k_d * x_err_dot

        x_dot_dot_command = p_term_x + d_term_x + acceleration_ff[0]

        b_x_c = x_dot_dot_command / c

        y_err = local_position_cmd[1] - local_position[1]
        y_err_dot = local_velocity_cmd[1] - local_velocity[1]

        p_term_y = k_p * y_err
        d_term_y = k_d * y_err_dot

        y_dot_dot_command = p_term_y + d_term_y + acceleration_ff[1]

        b_y_c = y_dot_dot_command / c
        #print("lateral_position_control:: b_x_c, b_y_c", b_x_c, b_y_c)
        return np.array([b_x_c, b_y_c]).clip(-1,1)
    
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehical's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)

        """
        self.rot_mat = euler2RM(*attitude) # unpack attitude 3-tuple
        b_z = self.rot_mat[2,2]
        z_k_p = 2
        z_k_d = 0.3

        z_err = altitude_cmd - altitude
        z_err_dot = vertical_velocity_cmd - vertical_velocity
        p_term = z_k_p * z_err          # proportional error
        d_term = z_k_d * z_err_dot      # derivative error

        u_1_bar = -p_term - d_term + acceleration_ff #PD controller
        c = (u_1_bar + GRAVITY) / b_z

        thrust = DRONE_MASS_KG * -c

        #print("altitude_control:: time= {0:.4f}, b_z, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, accel, thrust)".format(timer.time()),
        #     b_z, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff, thrust)

        return thrust


    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thrusts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """

        k_p_rollpitch = 10

        self.rot_mat = euler2RM (attitude[0], attitude[1],attitude[2])
        b_x = self.rot_mat[0, 2]
        b_x_err = -acceleration_cmd[0]/thrust_cmd - b_x # TODO how does this work?
        b_x_p_term = k_p_rollpitch * b_x_err

        b_y = self.rot_mat[1, 2]
        b_y_err = -acceleration_cmd[1]/thrust_cmd - b_y
        b_y_p_term = k_p_rollpitch * b_y_err
        b_x_commanded_dot = b_x_p_term
        b_y_commanded_dot = b_y_p_term

        rot_mat1=np.array([[self.rot_mat[1,0],-self.rot_mat[0,0]],[self.rot_mat[1,1],-self.rot_mat[0,1]]])/self.rot_mat[2,2]
        rot_rate = np.matmul(rot_mat1,np.array([b_x_commanded_dot,b_y_commanded_dot]).T)

        tau=2*DRONE_MASS_KG* 1.0
        p_c = rot_rate[0]/tau
        q_c = rot_rate[1]/tau


        print("roll_pitch_controller:: time= {0:.4f}, accel cmd, attitude, thrust_cmd, p_c, q_c)".format(timer.time()),
              acceleration_cmd, attitude, thrust_cmd, np.array([p_c, q_c]) )

        return np.array([p_c, q_c]).clip(-np.pi/12,np.pi/12)

    def body_rate_control(self, body_rate_cmd, body_rate): # Implementation reviewed (proportional gain controller in body frame with moment of inertia)
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:R
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2
        """
        body_p = 0.7

        u_bar_p = MOI[0] * (body_rate_cmd[0] - body_rate[0]) * body_p  # unit check: body_rate [rad/s^2] MOI [kg x m^2] Newton [kg*m/s^2]
        u_bar_q = MOI[1] * (body_rate_cmd[1] - body_rate[1]) * body_p  # assume rotation moments apply in the body frame
        u_bar_r = MOI[2] * (body_rate_cmd[2] - body_rate[2]) * body_p
        body_rate_cmd_adjusted = np.array([u_bar_p, u_bar_q, u_bar_r])

        #print("body_rate_control:: time= {0:.4f}, body_rate (cmd, curr, new)".format(timer.time()), body_rate_cmd, body_rate, body_new )
        #Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        return body_rate_cmd_adjusted



    def yaw_control(self, yaw_cmd, yaw): # Implementation reviewed, added numerical integration
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """
        yaw_p = 0.7
        psi_err = (yaw_cmd - yaw)

        # integrate over 40 samples, approx 1 second, to determine yaw acceleration correction needed
        self.accum_yaw_err +=  psi_err
        self.yaw_counter += 1
        yaw_counts_per_second = 4
        if self.yaw_counter % yaw_counts_per_second == 0 :
            self.accum_yaw_accel_update = self.accum_yaw_err / yaw_counts_per_second
            self.accum_yaw_err = 0
        r_c = yaw_p * self.accum_yaw_accel_update

        #print("yaw_control::time= {0:.4f}, yaw (cmd, curr, new)".format(timer.time()), yaw_cmd, yaw, r_c )
        return r_c

