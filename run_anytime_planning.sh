#!/bin/sh
# Define default arguments
start_x=50 start_y=30
goal_x=150 goal_y=150
robot_radius=3 clearance=2
method=0
inflation_factor=1.0
world_img="$PWD/images/check_img.png"

# Define help statement
usage="$(basename "$0") -- program to find path from start to goal in anytime fashion

Where:
    -h  show this help text
    
    -x  set the start x-coordinate (default: $start_x, type: int)
    
    -y  set the start y-coordinate (default: $start_y, type: int)
    
    -a  set the goal x-coordinate (default: $goal_x, type: int)
    
    -b  set the goal y-coordinate (default: $goal_y, type: int)
    
    -r  set radius of the robot (default: $robot_radius, type: int)
    
    -c  set clearance required between the robot and obstacles (default: $clearance, type: int)
    
    -m  set method the robot uses to find path from start to goal (default: $method, , type: int)
        Use 0: A*, 1: ATA*, 2: ARA*, 3: ANA*
        
    -i  set inflation factor for the method (default: $inflation_factor, type: float, minimum: 1.0)
    
Note: The world size is 300x200 so set the start and goal point accordingly."

# Start while loop to assign arguments
while getopts ":ha:b:c:i:m:r:x:y:" option; do
  case "$option" in
    a) goal_x=$OPTARG
       ;;
    b) goal_y=$OPTARG
       ;;
    c) clearance=$OPTARG
       ;;
    i) inflation_factor=$OPTARG
       ;;
    m) method=$OPTARG
       ;;
    r) robot_radius=$OPTARG
       ;;
    x) start_x=$OPTARG
       ;;
    y) start_y=$OPTARG
       ;;
    h) echo "$usage"
       exit
       ;;
  esac
done

# Create world if it doesn't exist
if [ ! -f "$world_img" ]; then
    python3 utils/create_world.py $start_x,$start_y $goal_x,$goal_y $robot_radius,$clearance
fi
# Compile and run the explorer to find path from start to goal
bash explorer.sh
cd build/
./explorer $start_x $start_y $goal_x $goal_y $method $inflation_factor $world_img
cd ../
# Create video to show path
python3 utils/create_video.py $start_x,$start_y $goal_x,$goal_y $robot_radius,$clearance
