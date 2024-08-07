import hebi
import os
from hebi.util import create_mobile_io
from time import sleep

def create_demo_from_config(lookup, example_config):

    # Load Mobile IO config as a dictionary
    mobile_io_dict = {}
    if 'mobile_io' in example_config.user_data:
        if type(example_config.user_data['mobile_io']) is dict:
            if set(example_config.user_data['mobile_io'].keys()) == set({"family", "name", "layout"}):
                if all(isinstance(example_config.user_data['mobile_io'].get(key), str) for key in set({"family", "name", "layout"})):
                    mobile_io_dict['family'] = example_config.user_data['mobile_io'].get('family')
                    mobile_io_dict['name'] = example_config.user_data['mobile_io'].get('name')
                    mobile_io_dict['layout'] = os.path.join(example_config.directory, example_config.user_data['mobile_io'].get('layout'))
                else:
                    raise TypeError('HEBI config "mobile_io" field must be a dictionary of strings, not parseable as strings')
            else:
                raise ValueError(f'HEBI config "mobile_io" field must contain exactly the keys: "family", "name", and "layout"')
        else:
            raise TypeError('HEBI config "mobile_io" field must be a dictionary of strings, not parseable as dictionary')

    # Load gripper config as a dictionary
    gripper_dict = {}
    if 'gripper' in example_config.user_data:
        if type(example_config.user_data['gripper']) is dict:
            if set(example_config.user_data['gripper'].keys()) == set({"family", "name", "gains", "close_effort", "open_effort"}):
                if all(isinstance(example_config.user_data['gripper'].get(key), str) for key in set({"family", "name", "gains"})) and \
                all(isinstance(example_config.user_data['gripper'].get(key), float) for key in set({"close_effort", "open_effort"})):
                    gripper_dict['family'] = example_config.user_data['gripper'].get('family')
                    gripper_dict['name'] = example_config.user_data['gripper'].get('name')
                    gripper_dict['gains'] = os.path.join(example_config.directory, example_config.user_data['gripper'].get('gains'))
                    gripper_dict['close_effort'] = example_config.user_data['gripper'].get('close_effort')
                    gripper_dict['open_effort'] = example_config.user_data['gripper'].get('open_effort')
                else:
                    raise TypeError('HEBI config "gripper" field must be a dictionary of strings for "family", "name" and "gains", and floats for "close_effort" and "open_effort", not parseable as some')
            else:
                raise ValueError(f'HEBI config "gripper" field must contain exactly the keys: "family", "gains", "close_effort", and "open_effort".')
        else:
            raise TypeError('HEBI config "gripper" field must be a dictionary of strings and floats, not parseable as dictionary')

    # Set up arm from config
    arm = hebi.arm.create_from_config(example_config)
    if arm is None:
        raise RuntimeError("Failed to create arm from config.")

    # Set up Mobile IO from config
    mobile_io = None
    if any(mobile_io_dict):
        print('Waiting for Mobile IO device to come online...')
        num_retries = 10

        for i in range(num_retries):
            mobile_io = create_mobile_io(lookup, mobile_io_dict['family'], mobile_io_dict['name'])

            if mobile_io is None:
                print(f"Couldn't find Mobile IO :( Check name, family, or device status.")
                sleep(1)
                if i == num_retries - 1:
                    raise RuntimeError("Failed to create Mobile IO from config.")

        mobile_io.send_layout(layout_file=mobile_io_dict['layout'])
        mobile_io.update()

    # Add the gripper
    gripper = None
    if any(gripper_dict):
        group = lookup.get_group_from_names([gripper_dict.get('family')], [gripper_dict.get('name')])
        if group is None:
            raise RuntimeError("Incorrect family or name of gripper")

        gripper = hebi.arm.Gripper(group, gripper_dict.get('close_effort'), gripper_dict.get('open_effort'))
        gripper.load_gains(gripper_dict.get('gains'))
        if not arm is None:
            arm.set_end_effector(gripper)

    return arm, mobile_io, gripper