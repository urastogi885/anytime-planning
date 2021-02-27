import ast
from sys import argv

from utils.robot_world import RobotWorld


script, robot_params = argv

if __name__ == '__main__':
    # Convert input arguments into required data types
    robot_params = tuple(ast.literal_eval(robot_params))
    # Initialize the robot world class
    robot_world = RobotWorld(robot_params[0], robot_params[1])
