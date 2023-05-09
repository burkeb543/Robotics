# Program to implement A* with wall avoidance from start to goal

from controller import Robot, Motor, DistanceSensor, Lidar
import math
import numpy as np
import time
from typing import Tuple, List
import csv


def turnLeft(wheels):
    wheels[0].setVelocity(-10.0)
    wheels[1].setVelocity(-10.0)
    wheels[2].setVelocity(10.0)
    wheels[3].setVelocity(10.0)

def turnRight(wheels):
    wheels[0].setVelocity(10.0)
    wheels[1].setVelocity(10.0)
    wheels[2].setVelocity(-10.0)
    wheels[3].setVelocity(-10.0)


def moveForward(wheels):
    wheels[0].setVelocity(10.0)
    wheels[1].setVelocity(10.0)
    wheels[2].setVelocity(10.0)
    wheels[3].setVelocity(10.0)
 

def stop(wheels):
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(0)
    wheels[2].setVelocity(0)
    wheels[3].setVelocity(0)
    

def initGps(TIME_STEP, robot):  # Initialise the GPS
    gps = []
    gpsNames = ['gps1', 'gps2']
    value = [[0, 0, 0], [0, 0, 0]]

    for i in range(len(gpsNames)):
        gps.append(robot.getDevice(gpsNames[i]))
        gps[i].enable(TIME_STEP)
    return gps, value


def initWheels(robot):  # Initialise the wheels
    wheels = []
    wheelsNames = ['front left wheel motor', 'back left wheel motor', 'front right wheel motor','back right wheel motor']

    for i in range(len(wheelsNames)):
        wheels.append(robot.getDevice(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(2.0)
    return wheels


def getGpsData(gps, value):  # Returns the GPS data from the sensors
    value[0] = gps[0].getValues()
    value[1] = gps[1].getValues()
    return value


def getRobotPose(gps_value):  # Returns the robots pose
    robot_location = [(gps_value[0][0] + gps_value[1][0]) / 2, (gps_value[0][1] + gps_value[1][1]) / 2]
    orientation_vector = [gps_value[0][1] - gps_value[1][1], -1 * (gps_value[0][0] - gps_value[1][0])]
    mag_orientation_vector = math.sqrt(pow(orientation_vector[0], 2) + pow(orientation_vector[1], 2))
    orientation_unit_vector = [0, 0]

    if mag_orientation_vector != 0:
        orientation_unit_vector[0] = orientation_vector[0] / mag_orientation_vector
        orientation_unit_vector[1] = orientation_vector[1] / mag_orientation_vector
    return robot_location, orientation_unit_vector


def getTargetOrientationVector(goal, robot_location):  # Calculates and returns the target orientation vector
    target_orientation_vector = [goal[0] - robot_location[0], goal[1] - robot_location[1]]
    mag_target_orientation_vector = math.sqrt(
        pow(target_orientation_vector[0], 2) + pow(target_orientation_vector[1], 2))
    target_orientation_unit_vector = [0, 0]

    if mag_target_orientation_vector != 0:
        target_orientation_unit_vector[0] = target_orientation_vector[0] / mag_target_orientation_vector
        target_orientation_unit_vector[1] = target_orientation_vector[1] / mag_target_orientation_vector
    return target_orientation_unit_vector


def createNodesList():  # Reads in csv file and stores thhe nodes in a list
    nodes = []
    with open('OccupancyGrid.csv', newline='') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',')
        for row in csvreader:
            node_row = []
            for element in row:
                occupied = int(element)
                g_cost = None
                h_cost = None
                f_cost = None
                parent = None
                node_row.append({"o": occupied, "g": g_cost, "h": h_cost, "f": f_cost, "p": parent})
            nodes.append(node_row)
    return nodes


def distBetweenPoints(x1, y1, x2, y2):  # Returns the distance between 2 points
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def moveToGoal(wheels, target, gpsValue, pose, target_pose, delta_theta):  # Moves robot to goal point
    # Calculate the distance and angle to the goal
    dist = distBetweenPoints(target[0], target[1], pose[0], pose[1])
    angle = math.atan2(target[1] - pose[1], target[0] - pose[0])

    # Check if the robot is facing the goal and move towards it
    if abs(delta_theta - angle) < math.pi / 2:
        moveForward(wheels)
    else:
        # Rotate to face the goal
        if delta_theta < angle:
            turnLeft(wheels)
        else:
            turnRight(wheels)


def get_neighbors(current_node, nodes):  # Returns neighbours in all 8 directions around robot
    neighbour_nodes_list = [{"row": current_node["row"] + 1, "column": current_node["column"]},
                            {"row": current_node["row"] + 1, "column": current_node["column"] + 1},
                            {"row": current_node["row"], "column": current_node["column"] + 1},
                            {"row": current_node["row"] - 1, "column": current_node["column"] + 1},
                            {"row": current_node["row"] - 1, "column": current_node["column"]},
                            {"row": current_node["row"] - 1, "column": current_node["column"] - 1},
                            {"row": current_node["row"], "column": current_node["column"] - 1},
                            {"row": current_node["row"] + 1, "column": current_node["column"] - 1}]
    return neighbour_nodes_list


def wall_cost(neighbor_node, nodes, wall_threshold): # Calculate a  wall cost for the neighbour
    row, column = neighbor_node["row"], neighbor_node["column"]
    cost = 0

    for direction in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]: # Iterate over each direction
        for distance in range(1, wall_threshold + 1): #  Iterate over each distance from the node for each direction
            r = row + distance * direction[0]
            c = column + distance * direction[1]
            if r < 0 or r >= len(nodes) or c < 0 or c >= len(nodes[0]):
                break
            if nodes[r][c]["o"] == 1:
                cost += (wall_threshold + 1 - distance) * 10 # Add a cost
                break

    return cost


        
def robotController():
    TIME_STEP = 64
    robot = Robot()
    
    robot.step(1)  # Call robot once to initialise gps values
    gps, gps_value = initGps(TIME_STEP, robot)
    gps_value = getGpsData(gps, gps_value)
    robot_location, orientation_Unit_Vector = getRobotPose(gps_value)

    nodes = createNodesList()

    goal_pos = {"x": 3, "y": -4}
    goal_node = {"row": int(goal_pos["y"] / -0.1), "column": int(goal_pos["x"] / 0.1)}

    start_pos = {"x": "undefined", "y": "undefined"}
    start_node = {"row": "undefined", "column": "undefined"}

    start_pos["x"], start_pos["y"] = robot_location[0], robot_location[1]
    start_node["row"], start_node["column"] = int(start_pos["y"] / (-0.1)), int(start_pos["x"] / (0.1))

    nodes[start_node["row"]][start_node["column"]]["g"] = 0
    dist_start_goal = distBetweenPoints(start_pos["x"], start_pos["y"], goal_pos["x"], goal_pos["y"]) / 0.1
    nodes[start_node["row"]][start_node["column"]]["h"] = dist_start_goal
    nodes[start_node["row"]][start_node["column"]]["f"] = dist_start_goal

    open_nodes_list = []
    closed_nodes_list = []
    open_nodes_list.append(start_node)

    while open_nodes_list: # while nodes remain in the open list
        current_node = min(open_nodes_list, key=lambda n: nodes[n["row"]][n["column"]]["f"])

        if current_node == goal_node:
            break

        open_nodes_list.remove(current_node)
        closed_nodes_list.append(current_node)
        neighbours = get_neighbors(current_node, nodes)

        for neighbor_node in neighbours:
            if neighbor_node in closed_nodes_list or nodes[neighbor_node["row"]][neighbor_node["column"]]["o"] == 1:
                continue

            WALL_THRESHOLD = 3
            neighbor_wall_cost = wall_cost(neighbor_node, nodes, WALL_THRESHOLD)
            tentative_g_score = neighbor_wall_cost + nodes[current_node["row"]][current_node["column"]][
                "g"] + distBetweenPoints(current_node["row"], current_node["column"], neighbor_node["row"],
                                         neighbor_node["column"])

            if neighbor_node not in open_nodes_list:
                open_nodes_list.append(neighbor_node)
            elif tentative_g_score >= nodes[neighbor_node["row"]][neighbor_node["column"]]["g"]:
                continue

            nodes[neighbor_node["row"]][neighbor_node["column"]]["p"] = current_node
            nodes[neighbor_node["row"]][neighbor_node["column"]]["g"] = tentative_g_score
            nodes[neighbor_node["row"]][neighbor_node["column"]]["h"] = distBetweenPoints(neighbor_node["row"],
                                                                                          neighbor_node["column"],
                                                                                          goal_node["row"],
                                                                                          goal_node["column"])
            nodes[neighbor_node["row"]][neighbor_node["column"]]["f"] = \
            nodes[neighbor_node["row"]][neighbor_node["column"]]["g"] + \
            nodes[neighbor_node["row"]][neighbor_node["column"]]["h"]

    goal_node_tuple = (int(goal_pos["y"] / -0.1), int(goal_pos["x"] / 0.1))

    path = []  # Create a path to traverse from start to  goal
    path_node = nodes[goal_node_tuple[0]][goal_node_tuple[1]]["p"]  # Set first node to goal node

    while path_node is not None:  # Work backwards from goal to  start node (stops when node has no parent)
        path.append(path_node)  # Add node to path
        path_node = nodes[path_node["row"]][path_node["column"]]["p"]  # Set next node to parent node
        path.append(path_node)  # Add parent node to path

    path.reverse()  # Reverse the path to now go from start to goal
    path.append(goal_node)  # Add the goal node to the path

    path_point_list = []  # Create a list to store the co-ordinates of the nodes
    for node in path[1:]:  # No need to include start point
        path_point_list.append(
            (node['column'] * 0.1, node['row'] * -0.1))  # Add co-ordinate from node and convert it to world scale

    wheels = initWheels(robot)  # Initialise wheels to start moving once the path has been established

    for point in path_point_list:  # For every point along the path to the goal
        temp_goal = point  # Set the point to be the current goal

        print(temp_goal)  # Display temporary goal for clarity
        mode = "MoveToGoal"  # Use a single equals sign to assign the value to mode

        while robot.step(TIME_STEP) != -1:
            gps_value = getGpsData(gps, gps_value)
            robot_location, orientation_Unit_Vector = getRobotPose(gps_value)
            target_Orientation_Unit_Vector = getTargetOrientationVector(temp_goal, robot_location)
            
            delta_theta = math.acos(np.dot(target_Orientation_Unit_Vector, orientation_Unit_Vector)) * (180 / math.pi)
            if math.asin(np.cross(target_Orientation_Unit_Vector, orientation_Unit_Vector)) < 0:
                delta_theta *= -1

            if abs(temp_goal[0] - robot_location[0]) < 0.1 and abs(temp_goal[1] - robot_location[1]) < 0.1:
                stop(wheels)
                print("Point Reached")
                mode == "Wait"
                break  # Exit the loop when the temporary goal is reached

            elif mode == "MoveToGoal":
                moveToGoal(wheels=wheels, target=temp_goal, gpsValue=gps_value, pose=orientation_Unit_Vector,
                           target_pose=target_Orientation_Unit_Vector, delta_theta=delta_theta)

    print("Goal Reached")
    stop(wheels)


if __name__ == "__main__":
    print("A* Pathfinding Algorithm - Reset to start")
    robotController()