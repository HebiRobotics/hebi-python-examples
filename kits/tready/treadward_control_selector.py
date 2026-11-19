"""Run the payload demo if the payload is present, else the plain base demo.

    python -m kits.tready.treadward_control_selector treadward.cfg.yaml treadward-core-sampler.cfg.yaml

Looks for the payload's modules; if found, runs the payload demo with both
configs, otherwise the base demo with the base config only.
"""

import subprocess
import sys
from time import sleep

import hebi

from kits.common.config import add_config_argument, config_arg_parser
from kits.tready.demo_runner import CONFIG_DIRS, load_config

PAYLOAD_DEMO = "kits.tready.smart_treadward_sampler_control"
BASE_DEMO = "kits.tready.treaded_base"
MAX_PAYLOAD_ATTEMPTS = 5


if __name__ == "__main__":
    parser = config_arg_parser("Run the payload demo if its modules are found, else the base demo", *CONFIG_DIRS)
    add_config_argument(parser, "base_config", "Treaded base config, e.g. treadward.cfg.yaml")
    add_config_argument(parser, "payload_config", "Payload config, e.g. treadward-core-sampler.cfg.yaml")
    args = parser.parse_args()
    payload_cfg = load_config(args.payload_config)

    lookup = hebi.Lookup()
    sleep(2)
    found = False
    for _ in range(MAX_PAYLOAD_ATTEMPTS):
        print("Looking for payload modules...")
        if lookup.get_group_from_names(payload_cfg.families, payload_cfg.names) is not None:
            found = True
            break
        sleep(1)

    command = [PAYLOAD_DEMO, args.base_config, args.payload_config] if found else [BASE_DEMO, args.base_config]
    print(f"Running {' '.join(command)}")
    subprocess.run([sys.executable, "-m", *command], check=False)  # the demo reports its own errors
