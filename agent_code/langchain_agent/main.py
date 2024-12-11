import agent_OOP #agent
import ros_tools #ros tools ROSA
import ros_robot_tools #ros tools for the simulation specifically


# imported tools from other files
tools = ros_tools.all_tools + ros_robot_tools.all_tools

#create agent
agent = agent_OOP.Gazebo_agent(tools=tools)

# infinite loop for user queries
while True:
    agent.query(input("Ask agent (or Ctrl+C to quit): "))

    #agent1.show_info() #this shows info about the current agent, it's a debugging feature

