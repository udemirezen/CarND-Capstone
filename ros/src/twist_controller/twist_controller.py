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

        target_v_lin_x = args[0]
        target_v_ang_z = args[1]
        current_v_lin_x = args[2]
        current_v_ang_z = args[3]

        steer_cmd = self.yaw_controller.get_steering(target_v_lin_x, target_v_ang_z, current_v_lin_x)

        velocity_diff = target_v_lin_x - current_v_lin_x;
        accelaration = velocity_diff / 0.5

        if accelaration > 0:
            accelaration = min(self.accel_limit, accelaration)
            throttle_cmd = accelaration / self.accel_limit
            brake_cmd = 0.0
        else:
            accelaration = max(self.decel_limit, accelaration)
            throttle_cmd = 0.0

            # Force=mass*acceleration,Torque=Force*radius
            # Torque = mass*acceleration*radius
            torque = self.vehicle_mass * accelaration * self.wheel_radius
            brake_cmd = abs(torque)

        return throttle_cmd, brake_cmd, steer_cmd
