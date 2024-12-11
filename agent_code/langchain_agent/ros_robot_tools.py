from langchain.agents import tool
from ros_tools import get_entities, execute_ros_command
import random as rd


class Gazebo_tools:

    @tool
    def give_robot_velocity(x_linear : float = 0.0, y_linear : float = 0.0,
                            z_linear : float = 0.0, x_angular : float = 0.0,
                            y_angular : float = 0.0, z_angular : float = 0.0,
                                rate : float = 1) -> dict:
        """
        Make the robot move with the given x,y,z linear velocities and x,y,z angular velocities

        :param x_linear: the linear velocity along the x axis that the robot will get.
        :param y_linear: the linear velocity along the y axis that the robot will get.
        :param z_linear: the linear velocity along the z axis that the robot will get.
        :param x_angular: the angular velocity along the x axis that the robot will get.
        :param y_angular: the angular velocity along the y axis that the robot will get.
        :param z_angular: the angular velocity along the z axis that the robot will get.
        :param rate: the rate, in Hz, at which this information will be passed to the robot.
        """
        cmd = f'ros2 topic pub' + ((' -r ' + str(rate)) if rate!=1 else '') + f' /cmd_vel \geometry_msgs/msg/Twist "{{linear: {{x: {x_linear}, y: {y_linear}, z: {z_linear}}}, angular: {{x: {x_angular}, y: {y_angular}, z: {z_angular}}}}}" --once' 
        
        success, output = execute_ros_command(cmd)

        if not success:
            return {"error": output}

        return {'output': output} #output of the terminal

    @tool
    def stop_robot() -> dict:
        """
        Stops the robot by setting all linear and angular velocities to 0.
        """
        cmd = f'ros2 topic pub' + f' /cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: 0, y: 0, z: 0}}, angular: {{x: 0, y: 0, z: 0}}}}" --once' 
        
        success, output = execute_ros_command(cmd)

        if not success:
            return {"error": output}

        return {'output': output} #output of the terminal

    @tool
    def ros2_topic_echo_info(
        topic: str,
        timeout: float = 1.0,
    ) -> dict:
        """
        Echoes the contents of a specific ROS2 topic, and always returns the message.

        :param topic: The name of the ROS topic to echo.
        :param timeout: Max time to wait for a message before timing out.

        :note: Do not use this tool if the number of messages is large.
            This will cause the response to be too large and may cause the tool to fail.
        """
        cmd = f"ros2 topic echo {topic} --once --spin-time {timeout}"

        success, output = execute_ros_command(cmd)

        if success : 
            return {"echoes": output}
        
        return {"error": output}

class Other_tools:
    @tool
    def random_num(
        maxi: float = 1.,
        mini: float = 0.,
    ) -> dict:
        """
        Returns a random number between the 2 specified numbers

        :param maxi: Maximum of the output.
        :param mini: Minimum of the output.
        """
        rand = rd.random()
        rand += mini
        rand *= maxi
        
        return {"random number": rand}


all_tools = [
    Gazebo_tools.give_robot_velocity,
    Gazebo_tools.stop_robot,
    Gazebo_tools.ros2_topic_echo_info,
    Other_tools.random_num
]
