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
GRAVITY = -9.81"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0
import time as timer

class NonlinearController(object):

    def __init__(self):
        """Initialize the controller object and control gains"""
        # Position
        #self.kpPosXY = 1.2 #1.5, set using omega, delta method
        self.kpPosZ  = 6.0 # 6.0
        self.kiPosXY = 0.3 #0.3
        self.kiPosZ  = 5.0 #20.0, not used

        # Velocity
        #self.kpVelXY = 2.2 #3.0
        self.kpVelZ  = 1.5 #1.5 , near critical damping
        self.kpAccFF = 0.05 #0.05

        # Angle
        self.kpBank = 6.05 # 10.
        self.kpYaw  = 4.5 #1.8

        # Angle rate
        self.kpPQR = np.array([20.0, 20.0,5]) #20,20,5

        self.dt = 0.025 # 25 ms sampling; not accurate and introducing variation between runs
        self.i_term = 0.1 # integrator term for altitude PID controller

        self.kpPosZ, self.kpVelZ = pid_values(self, 0.32, 0.9)
        print ("altitude PD", self.kpPosZ, self.kpVelZ)

        self.kpPosXY, self.kpVelXY = pid_values(self, 0.20, 0.6)
        print ("lateral PD", self.kpPosXY, self.kpVelXY )

        self.maxAccelXY     = 11.0 # m/s^2
        self.maxAscentRate  =  6.0 # m/s
        self.maxDescentRate =  2.0 # m/s
        self.maxTiltAngle   =  0.9 # radians
        self.maxVelocityXY  = 1.4 # m/s
        self.maxRollPitchRate = 2.9 # radians/sec
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
                               acceleration_ff = np.array([0,0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command

        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        err_p = local_position_cmd - local_position
        horiz_err = np.linalg.norm(err_p)
        err_dot = local_velocity_cmd - local_velocity
        velocity_magnitude = np.linalg.norm(err_dot)
#        if velocity_magnitude > self.maxVelocityXY :
        if horiz_err > 2:
            print("fail horiz", local_position, horiz_err, local_position_cmd)
            print("max velocity clipping at ", local_velocity, velocity_magnitude, local_velocity_cmd)
#            err_dot = err_dot / velocity_magnitude # scale velocity below maximum allowed

        return self.kpPosXY * err_p + self.kpVelXY * err_dot + acceleration_ff

    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)

        Returns: thrust command for the vehicle (+up)
        """
        self.rot_mat = euler2RM(*attitude) # unpack attitude 3-tuple
        b_z = self.rot_mat[2,2]

        z_err = altitude_cmd - altitude
        z_err_dot = vertical_velocity_cmd - vertical_velocity
        p_term = self.kpPosZ * z_err          # proportional error
        d_term = self.kpVelZ * z_err_dot      # derivative error
        self.i_term = self.i_term + z_err * self.dt #ignore integrator for now
        u_1_bar = self.kpPosZ * p_term + self.kpVelZ * d_term + acceleration_ff
        c = (u_1_bar - GRAVITY) / b_z
        thrust = DRONE_MASS_KG * c

        return thrust # return thrust +UP[N]


    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame

        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton

        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """

        if thrust_cmd <= 0. :
            return np.array([0.,0.])

        self.rot_mat = euler2RM(*attitude)

        c = -thrust_cmd / DRONE_MASS_KG
        #b_x_c, b_y_c = np.clip(acceleration_cmd / c, -1., 1.)

        #if abs(acceleration_cmd.any()) > self.maxAccelXY :
        #    print("clipping: unachievable lateral acceleration")
        #acceleration_clipped_cmd = acceleration_cmd.clip(-self.maxAccelXY, self.maxAccelXY)
        force_ratio_cmd = acceleration_cmd/c
        if abs(force_ratio_cmd.any()) > 1:
            print("clipping: unachievable lateral force ratio")
        force_cmd = force_ratio_cmd.clip(-1, 1)

        b_x = self.rot_mat[0, 2]
        b_x_c = np.arcsin(force_cmd[0])
        b_x_c = b_x_c.clip(-self.maxTiltAngle, self.maxTiltAngle)  # constrain commanded tilt (R[0,2])
        b_x_err = b_x_c - b_x
        b_x_p_term = self.kpBank * b_x_err
        b_y = self.rot_mat[1, 2]
        b_y_c = np.arcsin(force_cmd[1])
        b_y_c = b_y_c.clip(-self.maxTiltAngle, self.maxTiltAngle)
        b_y_err = b_y_c - b_y
        b_y_p_term = self.kpBank * b_y_err
        b_x_commanded_dot = b_x_p_term
        b_y_commanded_dot = b_y_p_term



        rot_mat1 = np.array([[self.rot_mat[1, 0], -self.rot_mat[0, 0]], [self.rot_mat[1, 1], -self.rot_mat[0, 1]]]) / \
                   self.rot_mat[2, 2]
        rot_rate = np.matmul(rot_mat1, np.array([b_x_commanded_dot, b_y_commanded_dot]).T)

        p_c = rot_rate[0]
        q_c = rot_rate[1]
        pq_cmd = np.array([p_c, q_c])
        #pq_cmd_clip = pq_cmd.clip(-self.maxRollPitchRate, self.maxRollPitchRate)
        #if pq_cmd_clip[0] != pq_cmd[0] or pq_cmd_clip[1] != pq_cmd[1]:
        #    print("clip roll_pitch", acceleration_cmd, pq_cmd)
        return pq_cmd


    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """


        pqrErr = body_rate_cmd - body_rate
        pqrErrAdj = np.multiply(pqrErr, np.transpose(self.kpPQR))
        pqrErrAdj = np.multiply(pqrErrAdj, np.transpose(MOI))
        body_frame_adjust = np.cross(body_rate, np.multiply(body_rate, np.transpose(MOI)))
        moment = pqrErrAdj + body_frame_adjust

        moment_magnitude = np.linalg.norm(moment)
        if moment_magnitude > MAX_TORQUE:
            moment = moment * moment_magnitude/MAX_TORQUE

        return pqrErrAdj + body_frame_adjust

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate

        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians

        Returns: target yawrate in radians/sec
        """
        yaw_error = yaw_cmd - yaw
        if yaw_error > np.pi:
            yaw_error = yaw_error - 2.0 * np.pi
        elif yaw_error < -np.pi:
            yaw_error = yaw_error + 2.0 * np.pi
        return self.kpYaw * yaw_error


def pid_values(self, t_rise, delta):
    """
    From `t_rise` and `delta` returns kp and kd
    """
    w = 1 / (1.57 * t_rise)
    return w * w, 2 * delta * w


MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0
#test

class NonlinearController(object):



    def __init__(self):
        """Initialize the controller object and control gains"""
        # Position
        self.kpPosXY = 1.5
        self.kpPosZ  = 6.0
        self.kiPosXY = 0.3
        self.kiPosZ  = 20.0

        # Velocity
        self.kpVelXY = 3.0
        self.kpVelZ  = 1.5
        self.kpAccFF = 0.05

        # Angle
        self.kpBank = 10.0
        self.kpYaw  = 1.8

        # Angle rate
        self.kpPQR = np.array([20.0, 20.0, 5.0])

        self.dt = 0.025 # 25 ms sampling

        self.maxAccelXY     = 11.0 # m/s^2
        self.maxAscentRate  =  6.0 # m/s
        self.maxDescentRate =  2.0 # m/s
        self.maxTiltAngle   =  1.0 # radians

        self.rot_mat = np.eye(3)
        self.start_time = timer.time()

        self.accum_yaw_err = 0  # integrator
        self.yaw_counter = 0
        self.accum_yaw_accel_update = 0  # current yaw acceleration target

        self.integrated_error_list = [0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0] # altitude integrator
        self.lat_integrated_error_list = [0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0] # latitude integrator
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
        #self.lat_integrated_error_list.append(x_err * self.dt)
        #self.lat_integrated_error_list.pop(0)
        #integrated_error = 0
        #for n in self.lat_integrated_error_list :
        #    integrated_error += n
        #i = integrated_error * self.kiPosXY


        x_err = local_position_cmd[0] - local_position[0]
        x_err_dot = local_velocity_cmd[0] - local_velocity[0]

        x_dot_dot_target = self.kpPosXY * x_err + self.kpVelXY * x_err_dot + acceleration_ff[0]

        y_err = local_position_cmd[1] - local_position[1]
        y_err_dot = local_velocity_cmd[1] - local_velocity[1]

        y_dot_dot_target = self.kpPosXY * y_err + self.kpVelXY * y_err_dot + acceleration_ff[1]

        phi_command = np.array([x_dot_dot_target, y_dot_dot_target])
        return phi_command

    
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


        z_err = altitude_cmd - altitude
        z_err_dot = vertical_velocity_cmd - vertical_velocity
        p_term = self.kpPosZ * z_err          # proportional error
        d_term = self.kpVelZ * z_err_dot      # derivative error






        self.integrated_error_list.append(z_err*self.dt)
        self.integrated_error_list.pop(0)
        integrated_error = 0
        for n in self.integrated_error_list :
            integrated_error += n

        i = integrated_error * self.kiPosZ

        u_1_bar = p_term + i + d_term + acceleration_ff #PD controller
        c = (u_1_bar + GRAVITY) / b_z

        thrust = DRONE_MASS_KG * c

        #print("altitude_control:: time= {0:.4f}, b_z, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, accel, thrust)".format(timer.time()),
        #     b_z, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff, thrust)
        #print("z_err, z_err_dot, p_term, d_term, accel_ff", z_err,z_err_dot,p_term,d_term, acceleration_ff)
        return thrust


    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thrusts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """


        self.rot_mat = euler2RM (attitude[0], attitude[1],attitude[2])
        c = -thrust_cmd/DRONE_MASS_KG

        b_x = self.rot_mat[0, 2]
        b_x_c = (acceleration_cmd[0]/c).clip(-self.maxTiltAngle, self.maxTiltAngle) # constrain commandeded tilt (R[0,2])
        b_x_err = b_x_c - b_x
        b_x_p_term = self.kpBank * b_x_err

        b_y = self.rot_mat[1, 2]
        b_y_c = (acceleration_cmd[1]/c).clip(-self.maxTiltAngle, self.maxTiltAngle)
        b_y_err = b_y_c - b_y
        b_y_p_term = self.kpBank * b_y_err
        b_x_commanded_dot = b_x_p_term
        b_y_commanded_dot = b_y_p_term

        rot_mat1=np.array([[self.rot_mat[1,0],-self.rot_mat[0,0]],[self.rot_mat[1,1],-self.rot_mat[0,1]]])/self.rot_mat[2,2]
        rot_rate = np.matmul(rot_mat1,np.array([b_x_commanded_dot,b_y_commanded_dot]).T)

        p_c = rot_rate[0]
        q_c = rot_rate[1]

       # print("roll_pitch_controller:: time= {0:.4f}, accel cmd, attitude, thrust_cmd, p_c, q_c)".format(timer.time()),
       #       acceleration_cmd, attitude, thrust_cmd, np.array([p_c, q_c]) )
        roll_pitch_clip_value = np.radians(70)
        if (np.array([p_c, q_c]) - np.array([p_c, q_c]).clip(-roll_pitch_clip_value,roll_pitch_clip_value)).any() :
            print ("CLIPPING roll_pitch")
        return np.array([p_c, q_c]).clip(-roll_pitch_clip_value,roll_pitch_clip_value)

    def body_rate_control(self, body_rate_cmd, body_rate): # Implementation reviewed (proportional gain controller in body frame with moment of inertia)
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:R
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2
        """


        u_bar_p = MOI[0] * (body_rate_cmd[0] - body_rate[0]) * self.kpPQR[0]  # unit check: body_rate [rad/s^2] MOI [kg x m^2] Newton [kg*m/s^2]
        u_bar_q = MOI[1] * (body_rate_cmd[1] - body_rate[1]) * self.kpPQR[1]  # assume rotation moments apply in the body frame
        u_bar_r = MOI[2] * (body_rate_cmd[2] - body_rate[2]) * self.kpPQR[2]
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

        yaw_error = yaw_cmd - yaw
        if yaw_error > np.pi:
            yaw_error = yaw_error - 2.0 * np.pi
        elif yaw_error < -np.pi:
            yaw_error = yaw_error + 2.0 * np.pi
        return self.kpYaw * yaw_error

