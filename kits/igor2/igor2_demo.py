import os
import sys

# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------

from components.configuration import Igor2Config
igor_config = Igor2Config('resources/config.yml')

# ------------------------------------------------------------------------------
# Running
# ------------------------------------------------------------------------------

from components.igor import Igor

igor = Igor(config=igor_config)
keep_running = True

def stop_running_callback(*args):
  global keep_running
  keep_running = False


igor.add_on_stop_callback(stop_running_callback)
igor.start()

# Mobile IO has been initialized once `igor.start()` returns
mio = igor.mobile_io
if mio is None:
  igor.request_stop()
  raise RuntimeError('Could not initialize controller for Igor.')

from time import sleep
while keep_running:
  sleep(1.0)
