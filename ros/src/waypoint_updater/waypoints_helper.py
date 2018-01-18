"""
Utilities used by waypoints_updater
"""

import numpy as np
import rospy
import copy


def get_waypoints_matrix(waypoints):
    """
    Converts waypoints listt to numpy matrix
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: 2D numpy array
    """

    waypoints_matrix = np.zeros(shape=(len(waypoints), 2), dtype=np.float32)

    for index, waypoint in enumerate(waypoints):
        waypoints_matrix[index, 0] = waypoint.pose.pose.position.x
        waypoints_matrix[index, 1] = waypoint.pose.pose.position.y

    return waypoints_matrix


def get_closest_waypoint_index(position, waypoints_matrix):
    """
    Given a pose and waypoints list, return index of waypoint closest to pose
    :param position: geometry_msgs.msgs.Position instance
    :param waypoints_matrix: numpy matrix with waypoints coordinates
    :return: integer index
    """

    x_distances = waypoints_matrix[:, 0] - position.x
    y_distances = waypoints_matrix[:, 1] - position.y

    squared_distances = x_distances**2 + y_distances**2
    return np.argmin(squared_distances)


def get_sublist(elements, start_index, size):
    """
    Given a list of elements, start index and size of sublist, returns
    sublist starting from start_index that has size elements. Takes care of wrapping around should
    start_index + size > len(elements)
    :param elements: list
    :param start_index: start index
    :param size: size of sublist
    :return: sublist, wrapped around beginning of elements list if necessary
    """

    # A very simple, not necessarily efficient solution
    doubled_elements = elements + elements[:size]
    return doubled_elements[start_index: start_index + size]


def get_sublist_covered(base_points, start_index, size_ahead, size_behind):
    """
    Given a list of elements, start index and size of sublist, returns
    sublist starting from start_index - size_behind, Takes care of wrapping around should
    start_index + size > len(elements)
    :param elements: list
    :param start_index: start index
    :param size: size of sublist
    :return: sublist, wrapped around beginning of elements list if necessary
    """

    behind_start_index = len(base_points) - size_behind
    extended_points = base_points[behind_start_index:] + base_points + base_points[:size_ahead]
    return extended_points[start_index: (start_index + size_ahead + size_behind)]


def get_smoothed_out_waypoints(waypoints):
    """
    Return smoothed out waypoints. Waypoints are smoothed out and evenly spaced out.
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: list of styx_msgs.msg.Waypoint instances
    """

    new_waypoints = [(waypoint.pose.pose.position.x, waypoint.pose.pose.position.y) for waypoint in waypoints]
    xs, ys = zip(*new_waypoints)
    degree = 3

    distances = []
    for first_element, second_element in zip(new_waypoints[:-1], new_waypoints[1:]):
        x_difference = second_element[0] - first_element[0]
        y_difference = second_element[1] - first_element[1]
        distance = np.sqrt(x_difference ** 2 + y_difference ** 2)
        distances.append(distance)

    distances.insert(0, 0)
    s_list = np.array(distances).cumsum()
    x_poly = np.polyfit(s_list, xs, degree)
    y_poly = np.polyfit(s_list, ys, degree)

    # evenly spaced out
    s_values = np.linspace(0, sum(distances), len(s_list))
    x_values = np.polyval(x_poly, s_values)
    y_values = np.polyval(y_poly, s_values)

    smooth_waypoints = []

    for index, waypoint in enumerate(waypoints):
        waypoint.pose.pose.position.x = x_values[index]
        waypoint.pose.pose.position.y = y_values[index]
        smooth_waypoints.append(waypoint)

    return smooth_waypoints


def save_waypoints(waypoints, path):

    waypoints_matrix = get_waypoints_matrix(waypoints)
    np.savetxt(path, waypoints_matrix)


def get_road_distance(waypoints):
    """
    Get road distance covered when following waypoints
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: float
    """

    total_distance = 0.0

    for index in range(1, len(waypoints)):

        x_distance = waypoints[index].pose.pose.position.x - waypoints[index - 1].pose.pose.position.x
        y_distance = waypoints[index].pose.pose.position.y - waypoints[index - 1].pose.pose.position.y

        distance = np.sqrt((x_distance**2) + (y_distance**2))

        total_distance += distance

    return total_distance


def get_braking_path_waypoints(waypoints, current_velocity, traffic_light_waypoint_id):
    """
    Get waypoints together with velocities such that car will stop at a traffic light
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :param current_velocity: current velocity in car heading direction: float
    :param traffic_light_waypoint_id: integer
    :return: list of styx_msgs.msg.Waypoint instances
    """

    offset = 0
    stop_id = traffic_light_waypoint_id - offset

    # Set target velocity to -1, to force car to brake to full stop. With velocity 0 braking from PID might not
    # be strong enough to really stop the car
    final_velocity = -0.2

    braking_waypoints = []

    # Slow down gradually to 0 from current waypoint to waypoint at little bit before traffic light
    for index, waypoint in enumerate(waypoints[:stop_id]):

        velocity = current_velocity + ((final_velocity - current_velocity) * float(index) / float(stop_id))

        # For velocities in front of stop light that would be very low, set them to a small positive value.
        # This is so we don't end up stopping too early, e.g. 5m before stop line
        rolling_velocity = 0.2
        velocity = max(rolling_velocity, velocity)

        waypoint.twist.twist.linear.x = velocity

        braking_waypoints.append(copy.deepcopy(waypoint))

    # For all further waypoints, set velocity to final_velocity
    for waypoint in waypoints[stop_id:]:

        waypoint.twist.twist.linear.x = final_velocity
        braking_waypoints.append(copy.deepcopy(waypoint))

    return braking_waypoints


def set_braking_behaviour(waypoints_ahead, braking_path_waypoints, current_position):
    """
    Given waypoints ahead and a braking path, set velocities for waypoints ahed
    :param waypoints_ahead: list of styx_msgs.msg.Waypoint instances
    :param braking_path_waypoints: list of styx_msgs.msg.Waypoint instances
    :param current_position: current car position
    """

    braking_path_waypoints_matrix = get_waypoints_matrix(braking_path_waypoints)

    # Establish where the car is in previously defined braking path
    car_waypoint_index_in_braking_path = get_closest_waypoint_index(current_position, braking_path_waypoints_matrix)

    braking_waypoints_count = min(len(waypoints_ahead), len(braking_path_waypoints[car_waypoint_index_in_braking_path:]))
    last_braking_path_index = car_waypoint_index_in_braking_path + braking_waypoints_count

    # Set braking velocities accordingly
    for index, braking_path_waypoint in enumerate(
            braking_path_waypoints[car_waypoint_index_in_braking_path:last_braking_path_index]):

        waypoints_ahead[index].twist.twist.linear.x = braking_path_waypoint.twist.twist.linear.x

    for waypoint in waypoints_ahead[braking_waypoints_count:]:
        waypoint.twist.twist.linear.x = -1


def get_smooth_waypoints_ahead(base_waypoints, car_position, look_ahead_waypoints_count, look_behind_waypoints_count):
    """
    Given base waypoints, car position and look ahead and behind count, compute a smooth path ahead of the car
    :param base_waypoints: list of styx_msgs.msg.Waypoint instances
    :param car_position: car position in base_waypoints
    :param look_ahead_waypoints_count: integer
    :param look_behind_waypoints_count: integer
    :return: list of styx_msgs.msg.Waypoint instances
    """

    waypoints_matrix = get_waypoints_matrix(base_waypoints)

    car_waypoint_index = get_closest_waypoint_index(car_position, waypoints_matrix)

    waypoints_ahead = get_sublist_covered(
        base_waypoints, car_waypoint_index, look_ahead_waypoints_count, look_behind_waypoints_count)

    smoothed_waypoints = get_smoothed_out_waypoints(waypoints_ahead)
    return smoothed_waypoints[look_behind_waypoints_count:]


def get_dynamic_smooth_waypoints_around_car(waypoints, car_position, look_ahead_metres, look_behind_metres):
    """
    Given base waypoints, car position and look ahead and behind metres, compute a smooth path around of the car.
    Path should stretch approximately look_behind_metres behind the carn and look_ahead_metres ahead of the car
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :param car_position: car position in base_waypoints
    :param look_ahead_metres: float, roughly how many metres ahead of the car we should return
    :param look_behind_metres: float, roughly how many metres behind the car we should consider to smooth the path
    :return: list of styx_msgs.msg.Waypoint instances
    """

    waypoints_matrix = get_waypoints_matrix(waypoints)
    car_waypoint_index = get_closest_waypoint_index(car_position, waypoints_matrix)

    waypoints_behind = get_waypoints_behind_car(waypoints, car_waypoint_index, look_behind_metres)
    waypoints_ahead = get_waypoints_ahead_of_car(waypoints, car_waypoint_index, look_ahead_metres)

    nearby_waypoints = waypoints_behind + waypoints_ahead
    smoothed_waypoints = get_smoothed_out_waypoints(nearby_waypoints)

    return smoothed_waypoints


def get_waypoints_ahead_of_car(waypoints, car_waypoint_index, distance):

    waypoints_ahead = waypoints[car_waypoint_index:] + waypoints

    index = int(distance)

    while get_road_distance(waypoints_ahead[:index]) < distance:

        index += 5

    return waypoints_ahead[:index]


def get_waypoints_behind_car(waypoints, car_waypoint_index, distance):

    waypoints_behind = waypoints + waypoints[:car_waypoint_index]

    shifted_car_index = car_waypoint_index + len(waypoints)

    index = shifted_car_index - int(distance)

    while get_road_distance(waypoints_behind[index:shifted_car_index]) < distance:

        index -= 5

    return waypoints_behind[index:shifted_car_index]


def is_traffic_light_ahead_of_car(waypoints, car_position, light_position):
    """
    Given waypoints segment, car position, light position and max distance, check if light is ahead of car.
    If light is ahead but appears to be outside of considered waypoints, then False is returned
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :param car_position: geometry_msgs.msgs.Point instance, current car position
    :param light_position: geometry_msgs.msgs.Point instance, traffic light position
    :return: bool
    """

    waypoints_matrix = get_waypoints_matrix(waypoints)

    car_index = get_closest_waypoint_index(car_position, waypoints_matrix)
    light_index = get_closest_waypoint_index(light_position, waypoints_matrix)

    # If light is ahead of car but not outside of waypoints, return True, else False
    return car_index < light_index and light_index != len(waypoints) - 1