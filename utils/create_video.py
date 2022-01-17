import ast
from sys import argv

from robot_world import RobotWorld

"""
Add various parameters as input arguments from user
:param start_pos: a tuple of 2 values: start coordinates (x, y)
:param goal_pos: a tuple of 2 values: goal coordinates (x, y)
:param robot_params: a tuple of 2 values: (robot radius, clearance)
"""
script, start_pos, goal_pos, robot_params = argv

if __name__ == "__main__":
    # Convert input arguments into required data types
    start_pos = tuple(ast.literal_eval(start_pos))
    goal_pos = tuple(ast.literal_eval(goal_pos))
    robot_params = tuple(ast.literal_eval(robot_params))
    # Initialize the robot world class
    robot_world = RobotWorld(start_pos, goal_pos, robot_params[0], robot_params[1])
    # Create video from path text files and delete the world image
    robot_world.create_video_animation()
    robot_world.remove_check_image()
