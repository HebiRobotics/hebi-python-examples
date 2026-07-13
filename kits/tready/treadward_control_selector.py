import subprocess
import sys
from time import sleep

import hebi

max_payload_attempts = 5

class PayloadInfo:
    def __init__(self, family: str, names: list[str], controller: str):
        self.family = family
        self.names = names
        self.controller = controller

payloads = [
    PayloadInfo("Treadward", ["Mast_Pivot", "Chain_Upper", "Chain_Lower", "Wiggly-IO"], 
                "kits.tready.smart_treadward_sampler_control"),
]
default_controller = "kits.tready.smart_treadward_control"


l = hebi.Lookup()
sleep(2)

payload_group = None
payload_controller = ""
attempts = 0
while attempts < max_payload_attempts and payload_group is None:
    attempts += 1
    print('Looking for payload modules...')
    sleep(1)

    for payload in payloads:
        payload_family = payload.family
        payload_names = payload.names
        payload_group = l.get_group_from_names(payload_family, payload_names)
        payload_controller = payload.controller
        if payload_group is not None:
            break


if payload_group is not None:
    print(f"Running {payload_controller}.py")
    subprocess.run([sys.executable, "-m", payload_controller])
else:
    print(f"Running {default_controller}.py")
    subprocess.run([sys.executable, "-m", default_controller])