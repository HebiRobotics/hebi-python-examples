import os
import sys

# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path

# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--imitation', action='store_true', default=False, dest='imitation',
                    help='Use the imitation group API to not connect to physical modules on the network.')
parser.add_argument('--mobile-io', action='store_true', default=False, dest='mobile_io',
                    help='Drive Igor using a Mobile IO app on the network. Requires --mobile-io-family and --mobile-io-name.')
parser.add_argument('--mobile-io-frequency', default=200.0, dest='mobile_io_freq',
                    help='Feedback frequency of Mobile IO group. Ignored if not controlling Igor using a Mobile IO device.')
parser.add_argument('--mobile-io-family', type=str, default=None, dest='mobile_io_family',
                    help='The Mobile IO app family.')
parser.add_argument('--mobile-io-name', type=str, default=None, dest='mobile_io_name',
                    help='The Mobile IO app name.')

args = parser.parse_args()

# TODO: Move into encapsulating function
io_fam = args.mobile_io_family
io_name = args.mobile_io_name
has_io_fam = io_fam is not None
has_io_name = io_name is not None
has_io = args.mobile_io or has_io_fam or has_io_name

if has_io:
  if not has_io_fam and has_io_name:
    print('Error: Invalid arguments.')
    parser.print_usage()
    exit(1)


from components.configuration import Igor2Config
igor_config = Igor2Config(args.imitation)

if has_io:
  from math import isfinite
  if args.mobile_io_freq < 1.0 or not isfinite(args.mobile_io_freq):
    print('ignoring specified feedback frequency {0}. Defaulting to 200Hz.'.format(args.mobile_io_freq))
    fbk_freq = 200.0
  else:
    fbk_freq = args.mobile_io_freq

  igor_config.select_controller_by_mobile_io(io_fam, io_name, fbk_freq)


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

# The joystick has been initialized once `igor.start()` returns
joy = igor.joystick
if joy is None:
  igor.request_stop()
  raise RuntimeError('Could not initialize controller for Igor.')

from time import sleep
while keep_running:
  sleep(1.0)
