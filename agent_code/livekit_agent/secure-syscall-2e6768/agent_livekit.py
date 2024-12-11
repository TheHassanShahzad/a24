import logging
import os

from dotenv import load_dotenv
from livekit.agents import (
    AutoSubscribe,
    JobContext,
    JobProcess,
    WorkerOptions,
    cli,
    llm,
)
from livekit.agents.pipeline import VoicePipelineAgent
from livekit.plugins import openai, deepgram, silero

import aiohttp
from typing import Annotated

# tool imports
import math
import time
import datetime

import ros_tools_mini

load_dotenv(dotenv_path=".env.local")
logger = logging.getLogger("voice-agent")


class AssistantFnc(llm.FunctionContext):
    # the llm.ai_callable decorator marks this function as a tool available to the LLM
    # by default, it'll use the docstring as the function's description
    @llm.ai_callable()
    async def get_weather(
        self,
        # by using the Annotated type, arg description and type are available to the LLM
        location: Annotated[
            str, llm.TypeInfo(
                description="The location to get the weather for")
        ],
    ):
        """Called when the user asks about the weather. This function will return the weather for the given location."""
        logger.info(f"getting weather for {location}")
        url = f"https://wttr.in/{location}?format=%C+%t"
        async with aiohttp.ClientSession() as session:
            async with session.get(url) as response:
                if response.status == 200:
                    weather_data = await response.text()
                    # response from the function call is returned to the LLM
                    # as a tool response. The LLM's response will include this data
                    return f"The weather in {location} is {weather_data}."
                else:
                    raise f"Failed to get weather data, status code: {response.status}"

    @llm.ai_callable()
    async def find_factorial(
        self,
        # by using the Annotated type, arg description and type are available to the LLM
        number: Annotated[
            int, llm.TypeInfo(
                description="the number the user wants to find the factorial of")
        ],
    ):
        """Called when the user asks to find the factorial of a number. This function will return the factorial of a number."""
        logger.info(f"finding the factorial of {number}")
        if number < 0:
            return f"cannot find the factorial of negative numbers"
        else:
            fact_number = math.factorial(number)
            return f"the factorial of {number} is {fact_number}"

    @llm.ai_callable()
    async def get_time(
        self,
    ):
        """Gets the number of seconds that passed since epoch (the first of january 1970)"""
        logger.info(f"Fetching time with get_time function")
        seconds = time.time()
        return (f"{str(seconds)}s have passed since 1st jan 1970")

    @llm.ai_callable()
    async def timestamp_convert(
        self,
        timestamp: Annotated[
            int, llm.TypeInfo(
                description="the number of seconds that passed since epoch (1st jan 1970)")
        ],
    ) -> str:
        """Converts passed timestamp into year,month,day,hour,minute,second format"""
        dt_object = datetime.date.fromtimestamp(timestamp)

        return str(dt_object)

    @llm.ai_callable()
    async def give_robot_velocity(
        self,
        x_linear: Annotated[
            int, llm.TypeInfo(
                description="the linear velocity along the x axis that the robot will get.")
        ] = 0.0,
        y_linear: Annotated[
            int, llm.TypeInfo(
                description="the linear velocity along the y axis that the robot will get.")
        ] = 0.0,
        z_linear: Annotated[
            int, llm.TypeInfo(
                description="the linear velocity along the z axis that the robot will get.")
        ] = 0.0,
        x_angular: Annotated[
            int, llm.TypeInfo(
                description="the angular velocity along the x axis that the robot will get.")
        ] = 0.0,
        y_angular: Annotated[
            int, llm.TypeInfo(
                description="the angular velocity along the y axis that the robot will get.")
        ] = 0.0,
        z_angular: Annotated[
            int, llm.TypeInfo(
                description="the angular velocity along the z axis that the robot will get.")
        ] = 0.0,
        rate: Annotated[
            int, llm.TypeInfo(
                description="the rate, in Hz, at which this information will be passed to the robot.")
        ] = 10,
    ) -> dict:
        """Make the robot move with the given x,y,z linear velocities and x,y,z angular velocities"""

        logger.info(f"called give_robot_velocity function")

        cmd = f'ros2 topic pub' + ((' -r ' + str(rate)) if rate != 1 else '') + \
            f' /cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: {x_linear}, y: \
                {y_linear}, z: {z_linear}}}, angular: {{x: {x_angular}, y: \
                    {y_angular}, z: {z_angular}}}}}" --once'

        success, output = ros_tools_mini.execute_ros_command(cmd)

        return 'success'

    @llm.ai_callable()
    async def stop_robot(
        self
    ) -> dict:
        """Make the robot move with the given x,y,z linear velocities and x,y,z angular velocities"""

        logger.info(f"called stop_robot function")

        cmd = f'ros2 topic pub' + \
            f' /cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: 0, y: 0, z: 0}}, angular: {{x: 0, y: 0, z: 0}}}}" --once'

        # logger.info(f"try to pass cmd of velocity function")

        success, output = ros_tools_mini.execute_ros_command(cmd)

        # logger.info(f"command called")

        return 'success'

    @llm.ai_callable()
    async def ros2_topic_echo_info(
        self,
        topic:  Annotated[
            str, llm.TypeInfo(description="The name of the ROS topic to echo.")
        ],
        timeout: Annotated[
            float, llm.TypeInfo(
                description="Max time to wait for a message before timing out.")
        ] = 3.0
    ) -> dict:
        """
        Echoes the contents of a specific ROS2 topic, and always returns the message.

        :note: Do not use this tool if the number of messages is large.
            This will cause the response to be too large and may cause the tool to fail.
        """

        logger.info(f"called ros2_topic_echo_info function")

        cmd = f"ros2 topic echo {topic} --once --spin-time {timeout}"

        success, output = ros_tools_mini.execute_ros_command(cmd)

        return output


def prewarm(proc: JobProcess):
    proc.userdata["vad"] = silero.VAD.load()


async def entrypoint(ctx: JobContext):
    initial_ctx = llm.ChatContext().append(
        role="system",
        text=(
            "You are a voice assistant created by LiveKit. Your interface with users will be voice. "
            "You should use short and concise responses, and avoiding usage of unpronouncable punctuation. "
            """You are a robot in a gazebo simulation. You have access to topics to know 
            the current state of the robot. 
            Prioritize ros2_topic_echo_info over ros2_topic_echo tool. 
            Here are some of the topics you have access to:
            '/camera/camera_info' gives you information on the calibration of the camera,
            '/camera/depth/camera_info' gives you information on the calibration of the depth camera,
            '/camera/depth/image_raw' gives you each pixel value for the image of te depth camera,
            '/clock' tells you how much time has passed inside the simulation,
            '/joint_states' gives you the current angle of each of the wheels,
            '/odom' gives you odometry of the robot: under 'pose:', 'pose:', 'position:', you can find the x y z position of the robot. By default, give 2 digit precision.
            '/scan' gives you information from the 2D ladar you have."""

        ),
    )

    logger.info(f"connecting to room {ctx.room.name}")
    await ctx.connect(auto_subscribe=AutoSubscribe.AUDIO_ONLY)

    # Wait for the first participant to connect
    participant = await ctx.wait_for_participant()
    logger.info(
        f"starting voice assistant for participant {participant.identity}")

    # This project is configured to use Deepgram STT, OpenAI LLM and TTS plugins
    # Other great providers exist like Cartesia and ElevenLabs
    # Learn more and pick the best one for your app:
    # https://docs.livekit.io/agents/plugins
    assistant = VoicePipelineAgent(
        vad=ctx.proc.userdata["vad"],
        stt=openai.STT.with_groq(),
        # stt=openai.STT.with_groq(model="distil-whisper-large-v3-en"),
        # llm=openai.LLM.with_groq(model="llama-3.2-90b-text-preview"), #deprecated
        llm=openai.LLM.with_groq(model="llama-3.3-70b-versatile"),

        tts=openai.TTS(),
        fnc_ctx=AssistantFnc(),
        chat_ctx=initial_ctx,
    )

    assistant.start(ctx.room, participant)

    # The agent should be polite and greet the user when it joins :)
    await assistant.say("Hey, how can I help you today?", allow_interruptions=True)


if __name__ == "__main__":
    cli.run_app(
        WorkerOptions(
            entrypoint_fnc=entrypoint,
            prewarm_fnc=prewarm,
        ),
    )
