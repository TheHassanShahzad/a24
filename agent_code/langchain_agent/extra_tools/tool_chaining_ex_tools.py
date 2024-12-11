from langchain.agents import tool
import time
from datetime import datetime

@tool
def get_time() -> int:
  """Gets the number of seconds that passed since the first of january 1970"""
  seconds = time.time()
  return seconds

@tool
def timestamp_convert(timestamp : int) -> str:
  """Converts passed timestamp into year,month,day,hour,minute,second """
  dt_object = datetime.fromtimestamp(timestamp)
  
  return str(dt_object)

all_tools = [
  get_time,
  timestamp_convert
]