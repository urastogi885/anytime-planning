import os
import ast
from sys import argv

from utils.robot_world import RobotWorld


script, robot_params = argv


if __name__ == "__main__":
    # Convert input arguments into required data types
    robot_params = tuple(ast.literal_eval(robot_params))
    # Initialize the robot world class
    robot_world = RobotWorld(robot_params[0], robot_params[1])
    EXPLORER_RUN_CMD = "cd build/ && ./explorer " + robot_world.CHECK_IMG_LOC
    # Run the explorer
    os.system(EXPLORER_RUN_CMD)

    robot_world.remove_check_image()
