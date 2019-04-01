# Igor Balancing Robot Python Demo

## Requirements

### Controller

The demo can be run using either a gamecontroller or Mobile IO device acessible from the computer running the demo.

* USB controller (Must be compatibile with SDL2 - check [here](https://github.com/gabomdq/SDL_GameControllerDB/blob/master/gamecontrollerdb.txt) for your controller)
* Mobile IO (Requires HEBI's Mobile IO app on an Android or iOS device)

### Software Requirements 
* [HEBI Python API](https://pypi.org/project/hebi-py/)
* [PySDL2](https://pypi.org/project/PySDL2/)
  * **Linux Users:** You should check if your distro has a package for this already. For Ubuntu, install the [pysdl2 package](https://launchpad.net/ubuntu/+source/pysdl2)
  * If not installing through a package manager, make sure you have the SDL2 library installed!

## Running

Simply run the `igor2_demo.py` file located at `kits/igor2`. Both Python 2 and 3 are supported, but note that Python 2 upstream will become [unsupported 1 Jan 2020.](https://pythonclock.org/)

Note: By default, the demo will look for a Mobile IO device with family `HEBI` and name `Mobile IO`. You must provide `--joystick` if you want to control Igor using a USB gamecontroller.

## Controls

The demo provides default mappings for both supported controllers. You can modify them, if needed, by editing the `components/configuration.py` file directly.

### Game Controller

The default gamecontroller mappings are as followed. Note that the image is of a generic Sony Playstation 4 controller, but the concept applies to all other SDL2 compatible controllers.
![ps4 igor](resources/ps4_igor.png)

*Note: To exit idle mode, press L3 (press in left axis stick)*

### Mobile IO

The default Mobile IO mappings are as followed. Note that the layout of the application may appear different on your device than what is shown, but the buttons and axes are guaranteed across any device.
![mobile io igor](resources/mobile_io_igor.png)
