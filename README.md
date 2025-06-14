# HEBI Python API Examples

This repository contains examples to go with the [online documentation](http://docs.hebi.us/tools.html#python-api) that help to get started with the HEBI Robotics APIs for Python. 

## Setup

All examples require the `hebi-py` pip package in order to run. You can install it through pip (_i.e._, `pip install hebi-py`).

It is expected that you have a `hebi-py` version of at least `2.2.0` for this repo.

**Note that this revision does not support Python 2!**

Additionally, some of the kits examples require the [PySDL2 Package](https://pypi.org/project/PySDL2/). You will need the `SDL2` shared library for this to work. For Windows, this library is included for you in this repository.

### Nightly API

If you want access to the newest API, you can download one of our nightly builds by doing:

`pip install http://docs.hebi.us/download/python/hebi-py-{version}.tar.gz`

Visit http://docs.hebi.us/download/python to see which specific versions are available for download, as they may change.

## Basic Examples

This folder contains examples that help you get started with the HEBI 
Robotics APIs for Python. There are separate examples for each of our different products:

* Actuator
* I/O Board
* Mobile I/O

The examples provided in each product work progressively through the following concepts:

* Lookup / Groups
* Feedback
* Commands
* Gains
* Trajectories
* Kinematics
* Example - Robot Arm

These API features and concepts are documented in more detail at:

* http://docs.hebi.us/tools.html#python-api
* http://docs.hebi.us/core_concepts.html

## Kits

This folder contains example code for various preconfigured kits. Additional documentation is available in the corresponding kit directories.

| Kit | Comment |
|-----|---------|
| [Arms](kits/arms) | A variety of arm configurations, from 4-DoF to 6-DoF. |
| [Igor](kits/igor2) | A 2-wheel, 2-arm, dynamic balancing mobile robot |

## Advanced Examples

This folder contains examples that show less commonly used concepts.
