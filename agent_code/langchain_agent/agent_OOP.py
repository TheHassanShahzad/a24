from langchain_openai import ChatOpenAI
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain.agents.format_scratchpad.openai_tools import format_to_openai_tool_messages
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain.agents import AgentExecutor
import os
import ros_tools
import ros_robot_tools

os.environ["OPENAI_API_KEY"] = "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"


_default_prompt = "You are a very powerful assistant, but don't know current events"
_default_llm_model = "gpt-4o-mini"

class Agent():
    def __init__(self, tools : list, prompt : str = _default_prompt,
                  llm_model : str = _default_llm_model):
        
        self.llm_model = llm_model
        self.prompt = prompt
        self.tools = tools

        self.llm = self.set_agent_llm()
        self.agent_prompt = self.set_prompt()
        self.llm_with_tools = self.bind_tools()

        self.agent = self.create_agent()

        self.agent_executor = AgentExecutor(
            agent=self.agent, 
            tools=self.tools, 
            verbose=True
            )
        
        print("*Agent_OOP* created new agent")

    def set_agent_llm(self):
        llm = ChatOpenAI(model=self.llm_model) #set llm
        return llm

    def set_prompt(self):
        agent_prompt = ChatPromptTemplate.from_messages(
            [
                (
                    "system",
                    str(self.prompt),
                ),
                ("user", "{input}"),
                MessagesPlaceholder(variable_name="agent_scratchpad"),
            ]
        ) #set prompt
        return agent_prompt

    def bind_tools(self):
        llm_with_tools = self.llm.bind_tools(self.tools) #set tools
        return llm_with_tools
    
    def create_agent(self):
        agent = (
            {
                "input": lambda x: x["input"],
                "agent_scratchpad": lambda x: format_to_openai_tool_messages(
                    x["intermediate_steps"]
                ),
            }
            | self.agent_prompt
            | self.llm_with_tools
            | OpenAIToolsAgentOutputParser()
        )
        return agent
    
    def __del__(self):
        """
        Deletes agent.
        """
        print("*Agent_OOP* deleted agent")
    
    def query(self, query:str):
        """
        Accepts a user query as a str, and feeds it to the agent. 
        Then the agent immeadiately starts answering the query (no 
        other user actions are necessary)
        """
        list(self.agent_executor.stream({"input": query}))

    def show_info(self):
        """
        Shows information about the agent.
        Print statements reveal the llm model, initial llm prompt,
        and the list of all the tools that the agent has access to.
        """
        print("************************")
        print("*Agent_OOP* Agent info :")

        print("llm model ->", self.llm_model)
        print("initial prompt ->", self.prompt)

        print("Tools :")
        i = 1 #counter
        for tool in self.tools:
            if i>1: print('') #skip line
            print(str(i) + '. ', end='') #number it
            print(tool) #show tool
            i += 1

        print("************************")

_gazebo_prompt = """
    You are a robot in a gazebo simulation. You have access to topics to know 
    the current state of the robot. 
    Prioritize ros2_topic_echo_info over ros2_topic_echo tool. 
    Here are some of the topics you have access to:
    '/camera/camera_info' gives you information on the calibration of the camera,
    '/camera/depth/camera_info' gives you information on the calibration of the depth camera,
    '/camera/depth/image_raw' gives you each pixel value for the image of te depth camera,
    '/clock' tells you how much time has passed inside the simulation,
    '/joint_states' gives you the current angle of each of the wheels,
    '/odom' gives you odometry of the robot: under 'pose:', 'pose:', 'position:', you can find the x y z position of the robot,
    '/scan' gives you information from the 2D ladar you have.
    """
_gazebo_tools = ros_tools.all_tools + ros_robot_tools.all_tools

class Gazebo_agent(Agent):
    def __init__(self, tools: list = _gazebo_tools, prompt : str = _gazebo_prompt,
                  llm_model : str = _default_llm_model):
        super().__init__(tools, prompt, llm_model)
