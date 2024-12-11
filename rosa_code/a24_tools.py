import os
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.agents import tool
from rosa import ROSA, RobotSystemPrompts
import subprocess
from typing import Tuple
from image_describer import analyze_image
from ament_index_python.packages import get_package_share_directory
import re

# Load environment variables
load_dotenv()

# Initialize OpenAI LLM using langchain_openai
openai_llm = ChatOpenAI(
    model_name="gpt-4o",  # Replace with your desired model
    temperature=0,        # Adjust as needed
    max_tokens=None,      # Set maximum token count (None for default)
    timeout=5,         # Optional timeout in seconds
    max_retries=2,        # Number of retries for API calls
    openai_api_key=os.getenv("OPENAI_API_KEY")  # API key from .env
)


def execute_ros_command(command: str) -> Tuple[bool, str]:
    """
    Execute a ROS2 command.

    :param command: The ROS2 command to execute.
    :return: A tuple containing a boolean indicating success and the output of the command.
    """

    # Validate the command is a proper ROS2 command
    cmd = command.split(" ")
    valid_ros2_commands = ["node", "topic", "service", "param", "doctor"]

    if len(cmd) < 2:
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[0] != "ros2":
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[1] not in valid_ros2_commands:
        raise ValueError(f"'ros2 {cmd[1]}' is not a valid ros2 subcommand.")

    try:
        output = subprocess.check_output(command, shell=True).decode()
        return True, output
    except Exception as e:
        return False, str(e)


@tool
def go_to_goal(
    x: float,
    y: float,
    theta: float
):
    """
    Move the robot to a specific point in the world with an x,y coordinate and angle theta.
    Check if the point is occupied before sending the goal

    :param x: The x coordinate of the point to move to in meters.
    :param y: The y coordinate of the point to move to in meters.
    :param theta: The angle to move to in radians.
    """

    cmd = f"ros2 topic pub /nav2_goal_pose std_msgs/Float64MultiArray \"data: [{x}, {y}, {theta}]\" --once"
    success,output = execute_ros_command(cmd)

    if success:
        print(f"Successfully sent command to move to ({x}, {y}, {theta})")
    else:
        print(f"Failed to send command ({x}, {y}, {theta}): {output}")
    
    
    # Implement the logic to move the robot to the specified point
    print(f"Moving robot to point: ({x}, {y}, {theta})")
    
@tool
def check_map_occupancy(x: float, y: float) -> bool:
    """
    Check if a specific point in the map is occupied. Called when a user wants to move the robot to a point. Cannot go to a point that is occupied.

    :output is a boolean true or fale. true means the coordinate is occupied. false means it isnt
    """

    cmd = f"ros2 service call /check_coordinate a24_interfaces/srv/CheckCoordinates \"{{x: {x}, y: {y}}}\""
    print(cmd)
    success, output = execute_ros_command(cmd)
  
    print(f"Checking map occupancy for point: ({x}, {y})")
    print(f"Output: {output}")

    is_occupied_val = "is_occupied=True" in output 
    return is_occupied_val #tells us if the map is occuped

@tool
def take_image():
    """
    Takes an image of what the robot is seeing in its camera and saves it as a .jpg image to this directory of the same "image.jpg"
    """
    cmd = f"ros2 service call /save_image std_srvs/srv/Trigger"
    success, output = execute_ros_command(cmd)

    print(f"Success: {success}")
    print(f"Output: {output}")

@tool
def describe_image() -> str:
    """
    Describes the image that the robot took. before using this tool take an image with the take_image tool to get the up to date image
    """

    cmd = f"ros2 service call /save_image std_srvs/srv/Trigger"
    success, output = execute_ros_command(cmd)
    
    package_name = 'a24'  # Replace with your package name
    package_path = get_package_share_directory(package_name)

    # Set the output directory to the "images" folder inside the package
    image_path = os.path.join(package_path, 'images')
    result = analyze_image(image_path + '/image.png')
    return result

@tool
def coord_2_depth(x: float, y: float) -> float:
    """
    Find the how far away the robot is from a point in the world by specifying an x,y coordinate and retrieving the depth value from the depth camera.
    Units of distance are metres and the coordinates are in pixels along and up the camera image.
    Specify the distance to 3 significant figures.
    """
    cmd = f"ros2 service call /calculate_distance a24_interfaces/srv/CoordDistance \"{{x: {x}, y: {y}}}\""
    success, output = execute_ros_command(cmd)


    if success:
        match = re.search(r"distance=([\d\.]+)", output)
        if match:
            return float(match.group(1))
        else:
            return float('nan')
    else:
        return f"Failed to calculate distance: {output}"

prompts = RobotSystemPrompts(
    embodiment_and_persona="You are are a 2 wheeled robot that can move around the house with the use of SLAM using a 2d lidar. you can see the world around you and you can move around the house. you can also navigate to a specific point in the house.",
    critical_instructions="Be sure to make conversions to the correct unit where necessary. Do not go to a point that is occupied in the map.",
    about_your_capabilities="You can move to a point in the world with an x,y coordinate and angle theta. You have a camera so you can take pictures and infer from the images. You also have a depth camera so you can find the distance to objects in the world.",
    mission_and_objectives="your mission is to at like a pet that follows instructions from the home owners"
)

# Pass the LLM to ROSA
rosa_instance = ROSA(
    ros_version=2,  # Specify your ROS version (1 or 2)
    llm=openai_llm,
    tools=[go_to_goal, check_map_occupancy, take_image, describe_image, coord_2_depth],
    prompts=prompts
)

# Use ROSA
# result = rosa_instance.invoke("check if the point 1,1 is occupied")
# result = rosa_instance.invoke("move to point 1,1")
# result = rosa_instance.invoke("take an image then describe it")
# result = rosa_instance.invoke("describe what you see")
result = rosa_instance.invoke("what is the distance of the coordinate 100,200 on the camera")


print(result)
