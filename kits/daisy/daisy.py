#!/usr/bin/env python3

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

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--imitation', action='store_true', default=False, dest='imitation',
                    help='Use the imitation group API to not connect to physical modules on the network.')
parser.add_argument('--mobile-io', action='store_true', default=True, dest='mobile_io', help='(ignored)')
parser.add_argument('--mobile-io-frequency', default=200.0, dest='mobile_io_freq',
                    help='Feedback frequency of Mobile IO group. Ignored if not controlling Igor using a Mobile IO device.')
parser.add_argument('--mobile-io-family', type=str, default=None, dest='mobile_io_family',
                    help='The Mobile IO app family.')
parser.add_argument('--mobile-io-name', type=str, default=None, dest='mobile_io_name',
                    help='The Mobile IO app name.')

args = parser.parse_args()

io_fam = args.mobile_io_family or 'HEBI'
io_name = args.mobile_io_name or 'mobileIO'

from components.configuration import HexapodConfig, Parameters
daisy_config = HexapodConfig(args.imitation)
daisy_parameters = Parameters()

from math import isfinite
if args.mobile_io_freq < 1.0 or not isfinite(args.mobile_io_freq):
  print('ignoring specified Mobile IO feedback frequency {0}. Defaulting to 200Hz.'.format(args.mobile_io_freq))
  fbk_freq = 200.0
else:
  fbk_freq = args.mobile_io_freq

daisy_config.select_controller_by_mobile_io(io_fam, io_name, fbk_freq)


# ------------------------------------------------------------------------------
# Running
# ------------------------------------------------------------------------------

from components.hexapod import Hexapod

daisy = Hexapod(config=daisy_config, params=daisy_parameters)
keep_running = True

def stop_running_callback(*args):
  global keep_running
  keep_running = False


daisy.add_on_stop_callback(stop_running_callback)
daisy.start()

# The joystick has been initialized once `daisy.start()` returns
joy = daisy.joystick
if joy is None:
  daisy.request_stop()
  raise RuntimeError('Could not initialize controller for Daisy.')

from time import sleep
while keep_running:
  sleep(1.0)
