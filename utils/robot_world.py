import os
import cv2
import shutil
import numpy as np
from glob import glob


def get_slopes(points) -> list:
    """
    Get slope of each edge of the polygon
    :param points: coordinates of the polygon
    :return: a list of slopes of the edges of the polygon
    """
    # Get no. of points
    points_length = len(points)
    i = 0
    # Define an empty list to store slopes of all edges
    slopes = []

    while i < points_length:
        # Get indices of the two points of the edge
        if i != points_length - 1:
            j = i + 1
        else:
            j = 0
        # Calculate slope and add it to the list
        slopes += (points[j][1] - points[i][1]) / (points[j][0] - points[i][0])
        i += 1

    return slopes


def get_y_values(x: int, slopes: list, coordinates, edge_count: int) -> list:
    """
    Calculate the y value of the current x from each edge
    :param x: x-coordinate of the current node
    :param slopes:a list of slopes of all edges of the polygon
    :param coordinates: a list of vertices of the polygon
    :param edge_count: no. of edges in the polygon
    :return: a list of all y-values
    """
    dist = []
    # Store all y-values
    for i in range(edge_count):
        dist += slopes[i] * (x - coordinates[i][0]) + coordinates[i][1]

    return dist

def get_path_list(dir: str) -> list:
    """
    List of path
    :param dir: Path to directory that stores path nodes
    :return: list of location of path text files
    """
    text_files_list = []
    # Add files to a list
    for file in glob(dir):
        text_files_list.append(file)

    # Sort the list and return it
    text_files_list.sort(key=lambda x:int(x.split('_')[1]))
    return text_files_list


class RobotWorld:
    def __init__(self, start: tuple, goal: tuple, radius: int, clearance: int) -> None:
        # Declare class constants
        self.WORLD_SIZE = 200, 300
        self.DEG_30 = np.pi / 6
        self.DEG_60 = np.pi / 3
        self.IMG_NAME = "robot_world.png"
        self.CHECK_IMG_NAME = "check_img.png"
        self.SAVE_DIR = "images"
        self.BUILD_DIR = "build"
        self.TEXT_FILES_DIR = "path"
        self.SAVE_LOC = os.path.join(os.getcwd(), self.SAVE_DIR, self.IMG_NAME)
        self.CHECK_IMG_LOC = os.path.join(os.getcwd(), self.SAVE_DIR, self.CHECK_IMG_NAME)
        self.TEXT_FILES_LOC = os.path.join(os.getcwd(), self.BUILD_DIR, self.TEXT_FILES_DIR)
        # Various class parameters
        self.height = self.WORLD_SIZE[0]
        self.width = self.WORLD_SIZE[1]
        self.thresh = radius + clearance
        self.start_pos = start
        self.goal_pos = goal
        # Get the robot's world
        self.world_img = self.draw_obstacles()

    def draw_circle(self) -> None:
        """
        Draw the circle obstacle on the map-image
        :param: nothing
        :return: nothing
        """
        # Define parameters of circular obstacles
        circle = [25, (225, 50)]
        # Define center of the circle
        a = circle[1][0]
        b = circle[1][1]
        # Define radius of the circle
        r = circle[0]
        # Draw the circle
        for y in range(self.height):
            for x in range(self.width):
                if (x - a) ** 2 + (y - b) ** 2 <= r ** 2:
                    self.world_img[y][x] = (0, 0, 0)

    def draw_ellipse(self) -> None:
        """
        Draw the circle obstacle on the map-image
        :param: nothing
        :return: nothing
        """
        # Define parameters of elliptical obstacles
        ellipse = [(40, 20), (150, self.height - 100)]
        # Get axes length of the ellipse
        a = ellipse[0][0]
        b = ellipse[0][1]
        # Get center of the ellipse
        center_a = ellipse[1][0]
        center_b = ellipse[1][1]
        # Draw the ellipse
        for y in range(self.height):
            for x in range(self.width):
                if ((x - center_a) / a) ** 2 + ((y - center_b) / b) ** 2 <= 1:
                    self.world_img[y][x] = (0, 0, 0)

    def draw_polygons(self) -> None:
        """
        Draw the convex polygon, rectangle and rhombus on the map-image
        :param: nothing
        :return: nothing
        """
        # Coordinates of the convex polygon
        coord_polygon = np.array([(20, self.height - 120),
                                  (25, self.height - 185),
                                  (75, self.height - 185),
                                  (100, self.height - 150),
                                  (75, self.height - 120),
                                  (50, self.height - 150)], dtype=np.int32)
        # Coordinates of the rectangle
        coord_rectangle = np.array([(95 - 75 * np.cos(self.DEG_30), self.height - 75 * np.sin(self.DEG_30) - 30),
                                    (95 - 75 * np.cos(self.DEG_30) + 10 * np.cos(self.DEG_60), self.height
                                     - 75 * np.sin(self.DEG_30) - 10 * np.sin(self.DEG_60) - 30),
                                    (95 + 10 * np.cos(self.DEG_60), self.height - 10 * np.sin(self.DEG_60) - 30),
                                    (95, self.height - 30)],
                                   dtype=np.int32).reshape((-1, 2))
        # Coordinates of the rhombus
        coord_rhombus = np.array([(300 - 75 - (50 / 2), self.height - (30 / 2) - 10),
                                  (300 - 75, self.height - 30 - 10),
                                  (300 - 75 + (50 / 2), self.height - (30 / 2) - 10),
                                  (300 - 75, self.height - 10)],
                                 dtype=np.int32).reshape((-1, 2))

        last_poly_slope = ((coord_polygon[2][1] - coord_polygon[5][1]) /
                           (coord_polygon[2][0] - coord_polygon[5][0]))

        # Get slopes of all the edges of the convex polygon, rectangle, and rhombus
        slopes_poly = get_slopes(coord_polygon)
        slopes_rect = get_slopes(coord_rectangle)
        slopes_rhombus = get_slopes(coord_rhombus)

        for y in range(self.height):
            for x in range(self.width):
                # Get y values for each edge of the convex polygon
                y_poly = get_y_values(x, slopes_poly, coord_polygon, 6)
                y_poly.append(last_poly_slope * (x - coord_polygon[5][0]) + coord_polygon[5][1])
                # Get y values for each edge of the rectangle
                y_rect = get_y_values(x, slopes_rect, coord_rectangle, 4)
                # Get y values for each edge of the rhombus
                y_rhom = get_y_values(x, slopes_rhombus, coord_rhombus, 4)
                # Draw the convex polygon
                if y_poly[0] <= y <= y_poly[6] and y_poly[1] <= y <= y_poly[5]:
                    self.world_img[y][x] = (0, 0, 0)
                elif y_poly[2] <= y <= y_poly[4] and y_poly[6] <= y <= y_poly[3]:
                    self.world_img[y][x] = (0, 0, 0)
                # Draw the tilted rectangle
                elif y_rect[0] <= y <= y_rect[2] and y_rect[1] <= y <= y_rect[3]:
                    self.world_img[y][x] = (0, 0, 0)
                # Draw the rhombus
                elif y_rhom[0] <= y <= y_rhom[3] and y_rhom[1] <= y <= y_rhom[2]:
                    self.world_img[y][x] = (0, 0, 0)

    def erode_image(self) -> bool:
        """
        Get eroded image to check for obstacles considering the robot radius and clearance
        :param: nothing
        :return: Status of file creation
        """
        # Get map with obstacles
        eroded_img = self.world_img.copy()
        eroded_img = cv2.cvtColor(eroded_img, cv2.COLOR_BGR2GRAY)
        # Erode map image for rigid robot
        if self.thresh:
            kernel_size = (self.thresh * 2) + 1
            erode_kernel = np.ones((kernel_size, kernel_size), np.uint8)
            eroded_img = cv2.erode(eroded_img, erode_kernel, iterations=1)
            # Include border in obstacle space
            for y in range(self.height):
                for x in range(self.width):
                    if (0 <= y < self.thresh or self.width - self.thresh <= x < self.width
                            or 0 <= x < self.thresh or self.height - self.thresh <= y < self.height):
                        eroded_img[y][x] = 0

        cv2.imwrite(self.CHECK_IMG_LOC, eroded_img)
        if not os.path.exists(self.CHECK_IMG_LOC):
            return False
        return True

    def remove_check_image(self) -> bool:
        """
        Remove check image from file system
        :param: nothing
        :return: Status of removal
        """
        os.remove(self.CHECK_IMG_LOC)
        if os.path.exists(self.CHECK_IMG_LOC):
            return False
        return True

    def draw_obstacles(self):
        """
        Draw map using half-plane equations
        :param: nothing
        :return: map-image with all obstacles
        """
        self.world_img = cv2.imread(self.SAVE_LOC)
        if self.world_img is None:
            # Initialize
            self.world_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            # Fill map-image with white color
            self.world_img.fill(255)
            # Draw various obstacles on the map
            self.draw_circle()
            self.draw_ellipse()
            self.draw_polygons()
            # Save the world to avoid re-creating at every run
            save_dir = os.path.join(os.getcwd(), self.SAVE_DIR)
            if not os.path.exists(save_dir):
                os.mkdir(save_dir)
            cv2.imwrite(self.SAVE_LOC, self.world_img)

        return self.world_img

    def create_video_animation(self) -> bool:
        """
        Show animation of map exploration and path from start to goal
        :param map_img: 2-d array with information of the map
        :return: nothing
        """
        # Add space between logs
        print("\n")

        # Get list of text file(s)
        text_files_list = get_path_list(self.TEXT_FILES_LOC + "/*.txt")
        if text_files_list is not None:
            print("[DEBUG] Text file(s) FOUND!")
            print("[INFO] Creating video...")

            # Define video-writer of open-cv to record the exploration and final path
            video_format = cv2.VideoWriter_fourcc(*'mp4v')
            video_output = cv2.VideoWriter('exploration' + '.mp4', video_format, 200.0,
                                        (self.WORLD_SIZE[1], self.WORLD_SIZE[0]))
            # Define various color vectors
            red = [0, 0, 255]
            blue = [255, 0, 0]
            green = [0, 255, 0]

            # Show path
            for file in text_files_list:
                map_img = self.world_img.copy()
                path_nodes = open(file, "r")
                path_nodes = path_nodes.readlines()
                start = path_nodes[0].split(",")
                goal = path_nodes[-1].split(",")
                for line in path_nodes:
                    node = line.split(",")
                    map_img[self.WORLD_SIZE[0] - int(node[1]), int(node[0])] = blue
                    video_output.write(map_img)

                # Draw start and goal node to the video frame in the form of filled circle
                cv2.circle(map_img, (int(start[0]), self.WORLD_SIZE[0] - int(start[1])), 3, green, -1)
                cv2.circle(map_img, (int(goal[0]), self.WORLD_SIZE[0] - int(goal[1])), 3, red, -1)

                # Show path for some time after exploration
                for _ in range(50):
                    video_output.write(map_img)

            # Free OpenCV objects
            video_output.release()
            cv2.destroyAllWindows()
            # Remove text files' directory
            shutil.rmtree(self.TEXT_FILES_LOC)

            # Check if video file was created
            if os.path.exists(os.path.join(os.getcwd(), "exploration.avi")):
                print("[DEBUG] Video creation SUCCESSFUL!")
                return True

            print("[DEBUG] Video creation UNSUCCESSFUL!")

        print("[DEBUG] Path file NOT FOUND!")
        return False
