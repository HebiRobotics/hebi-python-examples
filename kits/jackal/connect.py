#!/usr/bin/env python3

# Run with python 3!

from multiprocessing.connection import Client
import struct
import time

if __name__ == '__main__':
    conn = Client(('localhost', 6000), authkey=b'test')
    while True:
        linear = 0.0
        angular = 0.5
        bs = struct.pack('<f', linear) + struct.pack('<f', angular)
        conn.send_bytes(bs)
        time.sleep(0.01)

