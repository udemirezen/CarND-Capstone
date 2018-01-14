import rospy
from pid import PID
from lowpass import LowPassFilter
import numpy as np

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from  yaw_controller import YawController

class Controller(object):
    def __init__(self, *args, **kwargs):


        # TODO: Implement
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.wheel_radius = kwargs['wheel_radius']
        self.max_steer_angle = kwargs['max_steer_angle']

        self.yaw_controller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
                                            ONE_MPH, kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])

        self.throttle_pid_controller = pid.PID(kp=0.2, ki=0.01, kd=0.1, mn=self.decel_limit, mx=0.5 * self.accel_limit)
        self.brake_pid_controller = pid.PID(kp=30.0, ki=0.001, kd=5.0, mn=self.brake_deadband, mx=2000)
        self.steering_pid_controller = pid.PID(kp=1.0, ki=0.001, kd=0.5, mn=-self.max_steer_angle, mx=self.max_steer_angle)
        self.throttle_filter = lowpass.SmoothingFilter(window_weight=0.8)
        self.brake_filter = lowpass.SmoothingFilter(window_weight=0.0)
        self.steering_filter = lowpass.SmoothingFilter(window_weight=0.5)

        self.prev_time = rospy.get_rostime()

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        target_v_lin_x = args[0]
        target_v_ang_z = args[1]
        current_v_lin_x = args[2]
        current_v_ang_z = args[3]
        final_waypoints = args[4]
        current_pose = args[5]
        dbw_enabled = args[6]

        if not self.dbw_enabled:
            throttle_pid_controller.reset()
            brake_pid_controller.reset()
            steering_pid_controller.reset()
            return 0, 0, 0

        current_time = rospy.get_rostime()
        ros_duration = current_time - self.prev_time
        sample_time = ros_duration.secs + (1e-9 * ros_duration.nsecs)
        self.prev_time = current_time

        # Calculate the longitudinal and lateral error
        longitudinal_error = self.final_waypoints[1].twist.twist.linear.x - current_v_lin_x
        lateral_error = get_cross_track_error(final_waypoints, current_pose)

        # Too big longitudinal error reset the integrator to be able to break hard
        if longitudinal_error < -1.0:
            self.throttle_pid.error_integral = 0

        throttle_cmd = self.throttle_pid.step(linear_velocity_error, sample_time)
        throttle_cmd = self.throttle_filter.get_smoothed_value(throttle_cmd)

        brake_cmd = 0
        if throttle_cmd > 0:
            self.brake_filter.get_smoothed_value(0)
            self.brake_pid.reset()
        else:
            throttle_cmd = 0
            self.throttle_pid.reset()
            brake_cmd = self.brake_pid.step(-longitudinal_error, sample_time)
            brake_cmd = self.brake_filter.get_smoothed_value(brake_cmd)

        steer_cmd = self.steering_pid.step(lateral_error, sample_time)
        #steer_cmd = self.steering_filter.get_smoothed_value(steer_cmd)

        return throttle_cmd, brake_cmd, steer_cmd


def get_waypoints_coordinates_matrix(waypoints):
    """
    Given a list of waypoints, returns a numpy matrixes with x y coordinates
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: Nx2 numpy array where N is number of waypoints
    """

    points = []

    # Transform all points
    for waypoint in waypoints:

        x = waypoint.pose.pose.position.x
        y = waypoint.pose.pose.position.y

        points.append([x, y])

    return np.array(points)


def get_lateral_error(waypoints, current_pose):

    origin = waypoints[0].pose.pose.position

    # Put waypoints in an origin centered matrix
    points = []
    # Transform all points
    for waypoint in waypoints:
        x = waypoint.pose.pose.position.x
        y = waypoint.pose.pose.position.y
        points.append([x, y])

    waypoints_matrix = np.array(points) - np.array([origin.x, origin.y])

    # Calculate heading direction
    wp_offset = 10
    angle = np.arctan2(waypoints_matrix[wp_offset, 1], waypoints_matrix[wp_offset, 0])
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])

    # Rotate waypoints to origin
    rotated_waypoints = np.dot(waypoints_matrix, rotation_matrix)

    # Fit a polynomial to waypoints
    degree = 2
    coefficients = np.polyfit(rotated_waypoints[:, 0], rotated_waypoints[:, 1], degree)
    shifted_current_point = np.array([current_pose.position.x - origin.x, current_pose.position.y - origin.y])
    rotated_current_point = np.dot(shifted_current_point, rotation_matrix)

    expected_value = np.polyval(coefficients, rotated_current_point[0])
    actual_value = rotated_current_point[1]

    return -(actual_value - expected_value)