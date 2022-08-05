from audioop import add
from time import sleep
import os

import hebi

import typing
from numpy import true_divide

import requests
if typing.TYPE_CHECKING:
    from hebi._internal.group import Group
    from hebi._internal.mobile_io import MobileIO
    from hebi import Lookup

import os, re


class MioLayoutController:
    def __init__(self, mobile_io: 'MobileIO') -> None:
        self.address = 'http://10.10.10.160'

        self.mio = mobile_io
        print(self.address)

    def set_mobile_io_instructions(self, message):
        res = requests.post(self.address, message)
        print(res)