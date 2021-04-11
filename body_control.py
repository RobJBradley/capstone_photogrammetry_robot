
import numpy as np
import constant
import manipulator_control
from A_star import astar


#  ###############################################################################################################
# Function that controls the body to seek a position
#
# inputs:
#           np array of the origin of the body (3x1)
#           np array of the frame of the body (3x3)         i = forward along robot body, k = upwards in space
#           np array of obstacle grid obtained from the discretize_obj_vertices function
#
# result:
#           robot moves the body to the desired location, iterating around the obstacle grid
#
#  ###############################################################################################################
def body_seek(od_body, cd_body, obstacle_grid):

    # get current positions and orientations from udp packet
    if constant.using_webots:
        [oc_body, cc_body, oc_head, cc_head] = manipulator_control.get_webots_data(np.array([0, 0, 0, 0]),
                                                                                   np.array([0, 0, 0, 0]))
    else:
        [oc_body, cc_body, oc_head, cc_head] = manipulator_control.get_robot_data()

    # start point and goal
    # real start = x,y currently -> converted into a discretized array position
    real_start = (oc_body[0, 0] // constant.X_ROOM_DISCRETIZATION, oc_body[1, 0] // constant.Y_ROOM_DISCRETIZATION)
    # shift values to all positive grid values
    start = (real_start[0] + constant.OBSTACLE_GRID_SIZE_X // 2, real_start[1] + constant.OBSTACLE_GRID_SIZE_Y // 2)

    # real goal = x,y desired -> converted into a discretized array position
    real_goal = (od_body[0, 0] // constant.X_ROOM_DISCRETIZATION, od_body[1, 0] // constant.Y_ROOM_DISCRETIZATION)
    # shift values to all positive grid values
    goal = (real_goal[0] + constant.OBSTACLE_GRID_SIZE_X // 2, real_goal[1] + constant.OBSTACLE_GRID_SIZE_Y // 2)

    print("start and goal", start, goal)

    # run the route
    route = astar(obstacle_grid, start, goal)

    print("finished A*")

    # adjust the goal to not be obstructed if the result is false
    while not route:
        # prioritize away from the center
        if abs(real_goal[0]) == 0:
            add_x = 1
        else:
            add_x = real_goal[0] // abs(real_goal[0])  # add +/- x to the goal
        # prioritize away from the center
        if abs(real_goal[1]) == 0:
            add_y = 1
        else:
            add_y = real_goal[1] // abs(real_goal[1])  # add +/- y to the goal

        # adjust the goal to not be obstructed
        goal = (goal[0] + add_x, goal[1] + add_y)
        # recalculate with the new goal
        route = astar(obstacle_grid, start, goal)

    route = route[::-1]  # A* returns the points in reverse order, so we inverse the order

    # abridge the route - remove all points along a straight line except for the end point
    last_direction = [0, 0]
    abridged_route = []
    for increment in range(len(route) - 1):
        direction = [route[increment + 1][0] - route[increment][0],
                     route[increment + 1][1] - route[increment][1]]
        if direction != last_direction:
            abridged_route.append((route[increment][0], route[increment][1]))
            last_direction = direction

    # loop above does not include the goal in the new list
    abridged_route.append(goal)

    # shift all positive grid values to real values in space
    for i in range(len(abridged_route)):
        abridged_route[i] = (abridged_route[i][0] - constant.OBSTACLE_GRID_SIZE_X // 2,
                             abridged_route[i][1] - constant.OBSTACLE_GRID_SIZE_Y // 2)
    # print(abridged_route)

    # Loop until error is within the threshold
    while True:
        # get current positions and orientations from udp packet
        if constant.using_webots:
            [oc_body, cc_body, oc_head, cc_head] = manipulator_control.get_webots_data(np.array([0, 0, 0, 0]),
                                                                                       np.array([0, 0, 0, 0]))
        else:
            [oc_body, cc_body, oc_head, cc_head] = manipulator_control.get_robot_data()

        # body_pid algorithm
            # rotate robot
            # match i_direction of cd_body to (next_point in abridged_route - current_location) vector
            # use kahan_p2 to solve the angle error, and apply a PID to both wheels
            #                                        using (wheel_right_PWM == - wheel_left_PWM)
            # until the angle error = 0

            # move in straight line
            # calculate the distance error between current_location and next_point in abridged_route
            # apply PID to both wheels (left == right) to drive in a straight line
        # repeat until at last point then break

    #

    return


#  ###############################################################################################################
# obstacle grid discretization algorithm
# parse .obj for all vertices
# Used before the body_seek function
# input: a list of vertices from the .obj file [[x1, y1, z1], [x2, y2, z2], ..., [xn, yn, zn]]
# output: np array of an obstacle grid with value 0 with no obstacle and 1 in place for an obstacle
#  ###############################################################################################################
def discretize_obj_vertices(vertex):
    obstacle_grid = np.zeros((constant.OBSTACLE_GRID_SIZE_X, constant.OBSTACLE_GRID_SIZE_Y))
    for i in vertex:
        # discretize the vertices
        discrete_vertex_x = i[0] // constant.X_ROOM_DISCRETIZATION
        discrete_vertex_y = i[1] // constant.Y_ROOM_DISCRETIZATION
        # shift vertices to all positive values
        array_index_x = int(discrete_vertex_x + constant.OBSTACLE_GRID_SIZE_X // 2)
        array_index_y = int(discrete_vertex_y + constant.OBSTACLE_GRID_SIZE_Y // 2)
        # print("vertex", discrete_vertex_x, discrete_vertex_y)
        # print("index", array_index_x, array_index_y)
        obstacle_grid[array_index_x, array_index_y] = 1

    # print(obstacle_grid)
    return obstacle_grid
