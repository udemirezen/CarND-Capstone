#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, Twist

import styx_msgs.msg
import geometry_msgs.msg
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.use_pid_control = False
        self.controller = Controller(use_pid_control=self.use_pid_control,
            wheel_base=wheel_base, steer_ratio=steer_ratio,  
            max_lat_accel=max_lat_accel, max_steer_angle=max_steer_angle,
            accel_limit=accel_limit, decel_limit=decel_limit,
            vehicle_mass=vehicle_mass, fuel_capacity=fuel_capacity,
            brake_deadband=brake_deadband, wheel_radius=wheel_radius )
        self.controller.reset()

        # TODO: Subscribe to all the topics needed!
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/final_waypoints', styx_msgs.msg.Lane, self.final_waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', geometry_msgs.msg.PoseStamped, self.current_pose_cb, queue_size=1)

        self.dbw_enabled = False
        self.cur_vel = Twist()
        self.twist_cmd = Twist()
        self.final_waypoints = None
        self.cur_pose = None
        self.prev_time = rospy.get_rostime()


        self.loop()

    def loop(self):
        rate = rospy.Rate(20) # For 50Hz operation requirement
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled

            current_time = rospy.get_rostime()
            ros_duration = current_time - self.prev_time
            sample_time = ros_duration.secs + (1e-9 * ros_duration.nsecs)
            self.prev_time = current_time

            data = [self.twist_cmd, self.cur_vel, self.cur_pose, self.final_waypoints]
            data_availabe = all([x is not None for x in data])


            if self.dbw_enabled is True and data_availabe:
                throttle, brake, steering = self.controller.control(
                    self.twist_cmd.linear.x,
                    self.twist_cmd.angular.z,
                    self.cur_vel.linear.x,
                    self.cur_vel.angular.z,
                    self.final_waypoints,
                    self.cur_pose,
                    sample_time)

                self.publish(throttle, brake, steering)
            else:
                print("NO DATA")
            rate.sleep()

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = bool(msg.data)
        if self.dbw_enabled is False:
            self.controller.reset()

    def current_velocity_cb(self, msg):
        self.cur_vel = msg.twist

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg.twist

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg.waypoints

    def current_pose_cb(self, msg):
        self.cur_pose = msg.pose

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake

        if brake == 0:
            self.throttle_pub.publish(tcmd)
            self.steer_pub.publish(scmd)
        if throttle == 0:
            self.brake_pub.publish(bcmd)
            self.steer_pub.publish(scmd)


if __name__ == '__main__':
    DBWNode()
