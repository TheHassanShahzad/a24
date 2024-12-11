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
 
#tool imports
import math
import time 
import datetime

import ros_tools_mini
 
load_dotenv(dotenv_path=".env.local")
logger = logging.getLogger("voice-agent")
 


class LiveKitTools(llm.FunctionContext):
    # the llm.ai_callable decorator marks this function as a tool available to the LLM
    # by default, it'll use the docstring as the function's description
    @llm.ai_callable()
    async def get_weather(
        self,
        # by using the Annotated type, arg description and type are available to the LLM
        location: Annotated[
            str, llm.TypeInfo(description="The location to get the weather for")
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
            int, llm.TypeInfo(description="the number the user wants to find the factorial of")
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
        

    # @llm.ai_callable()
    # async def get_time(
    #     self, 
    # ) -> int:
    #     """Gets the number of seconds that passed since the first of january 1970"""
    #     logger.info(f"Fetching time with get_time function")
    #     seconds = time.time()
    #     return seconds

    @llm.ai_callable()
    async def timestamp_convert(
        timestamp : Annotated[
            int, llm.TypeInfo(description="the number of seconds that passed since epoch (1st jan 1970)")
        ],
        ) -> str:
        """Converts passed timestamp into year,month,day,hour,minute,second format"""
        dt_object = datetime.date.fromtimestamp(timestamp)

        return str(dt_object)
    
    @llm.ai_callable()
    async def give_robot_velocity(
        self,
        x_linear : Annotated[
            int, llm.TypeInfo(description="the linear velocity along the x axis that the robot will get.")
        ] = 0.0,
        y_linear: Annotated[
            int, llm.TypeInfo(description="the linear velocity along the y axis that the robot will get.")
        ] = 0.0,
        z_linear: Annotated[
            int, llm.TypeInfo(description="the linear velocity along the z axis that the robot will get.")
        ] = 0.0,
        x_angular: Annotated[
            int, llm.TypeInfo(description="the angular velocity along the x axis that the robot will get.")
        ] = 0.0,
        y_angular: Annotated[
            int, llm.TypeInfo(description="the angular velocity along the y axis that the robot will get.")
        ] = 0.0,
        z_angular: Annotated[
            int, llm.TypeInfo(description="the angular velocity along the z axis that the robot will get.")
        ] = 0.0,
        rate: Annotated[
            int, llm.TypeInfo(description="the rate, in Hz, at which this information will be passed to the robot.")
        ] = 10,
        ) -> dict:

        """Make the robot move with the given x,y,z linear velocities and x,y,z angular velocities"""

        logger.info(f"tried to call velocity function")
        
        cmd = f'ros2 topic pub' + ((' -r ' + str(rate)) if rate!=1 else '') + f' /cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: {x_linear}, y: {y_linear}, z: {z_linear}}}, angular: {{x: {x_angular}, y: {y_angular}, z: {z_angular}}}}}" --once' 

        logger.info(f"***Hagent*** => tried to run {cmd}")

        success, output = ros_tools_mini.execute_ros_command(cmd)

        if not success:
            return {"error": output}

        return {'output': output}

    