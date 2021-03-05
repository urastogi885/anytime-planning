import os
import ast
from sys import argv

import utils.robot_world
from utils.robot_world import RobotWorld

"""
Add various parameters as input arguments from user
:param start_pos: a tuple of 2 values: start coordinates (x, y)
:param goal_pos: a tuple of 2 values: goal coordinates (x, y)
:param robot_params: a tuple of 2 values: (robot radius, clearance)
:param method: Path finding method to be used
:param epsilon: Inflation for the method
"""
script, start_pos, goal_pos, robot_params, method, epsilon = argv


if __name__ == "__main__":
    # Convert input arguments into required data types
    start_pos = tuple(ast.literal_eval(start_pos))
    goal_pos = tuple(ast.literal_eval(goal_pos))
    robot_params = tuple(ast.literal_eval(robot_params))
    # Initialize the robot world class
    robot_world = RobotWorld(start_pos, goal_pos, robot_params[0], robot_params[1])

    # Run the explorer
    EXPLORER_RUN_CMD = "cd build/ && ./explorer " + (str(start_pos[0]) + " "
                        + str(start_pos[1]) + " " + str(goal_pos[0]) + " "
                        + str(goal_pos[1]) + " " + str(method) + " "
                        + str(epsilon) + " " + robot_world.CHECK_IMG_LOC)
    os.system(EXPLORER_RUN_CMD)

    robot_world.create_video_animation()

    robot_world.remove_check_image()
