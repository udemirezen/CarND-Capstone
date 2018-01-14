import rospy

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
        self.yaw_controller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
                                            ONE_MPH, kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        target_velocity_linear_x = args[0]
        target_velocity_angular_z = args[1]
        current_velocity_linear_x = args[2]
        current_velocity_angular_z = args[3]

        steer_cmd = self.yaw_controller.get_steering(
            target_velocity_linear_x, target_velocity_angular_z, current_velocity_linear_x)

        diff_vel = target_velocity_linear_x - current_velocity_linear_x;
        accel = diff_vel / 0.5

        if accel > 0:
            accel = min(self.accel_limit, accel)
            throttle_cmd = accel / self.accel_limit
            brake_cmd = 0.0
        else:
            accel = max(self.decel_limit, accel)
            throttle_cmd = 0.0
            # Force=mass*acceleration,Torque=Force*radius
            # Torque = mass*acceleration*radius
            torque = self.vehicle_mass * accel * self.wheel_radius
            brake_cmd = abs(torque)

        return throttle_cmd, brake_cmd, steer_cmd
