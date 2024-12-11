from langchain.agents import tool
import time as time
import turtle

"""
Those tools use the turtle package in python. They are not related to ros2 or
our Gazebo simulation. 
We developed those tools at the begining of our project when we were 
experimenting with LLMs and tools and didn't have a gazebo simulation yet. 
"""



@tool 
def turtle_move_forward(distance : int) -> None:
  """Make the turtle move according to its orientation by the distance passed"""
  turtle.forward(distance)
  return None

@tool 
def turtle_get_state() -> dict:
  """Returns a dictionary with the x position, y position and orientation of the turtle. 
  The orientation is in degrees and the turtle points to the right when orientation is 0.
  """
  x, y = turtle.pos()
  orientation = turtle.heading()

  dico = {
     'x' : x,
     'y' : y,
     'orientation' : orientation
  }

  return dico

@tool 
def turtle_goto(x: int, y: int, orientation: float) -> None:
   """Makes the turtle go to the passed x and y coordinates, and gives it the passed orientation. 
   The passed x value must be between -350 and 350 and the passed y value must be between -300 and
   300. If the turtle's pen is down, it will draw this movement."""
   turtle.goto(x, y)
   turtle.setheading(orientation)

@tool 
def turtle_pen_down() -> None:
   """When this function is called, the turtle will start drawing in its path"""
   turtle.down()

@tool
def turtle_pen_up() -> None:
   """When this function is called, the turtle will stop drawing in its path"""
   turtle.up()

@tool 
def turtle_orientation(orientation : float) -> None:
   """Changes the turtle's orientation to the passed orientation"""
   turtle.setheading(orientation)


turtle_tools = [
   turtle_move_forward,
   turtle_get_state,
   turtle_goto,
   turtle_pen_down,
   turtle_pen_up,
   turtle_orientation
]

all_tools = turtle_tools