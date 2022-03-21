#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from multiprocessing.connection import Listener
from array import array
import struct
import threading

class HebiJoySpoofer:
    def __init__(self):
        self.tv = Twist()
        self.ready = False
        rospy.init_node('hebi_joy_bridge_py2', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.listener = None
        self.is_shutdown = False

    def shutdown(self):
        print("Shutting down")
        self.is_shutdown = True

    def startListener(self):
        self.listener = threading.Thread(target=self.runListener, args=())
        self.listener.daemon = True
        self.listener.start()

    def runListener(self):
        conn_listener = Listener(('localhost', 6000), authkey=b'test')
        while not rospy.is_shutdown() and not self.is_shutdown:
            conn = conn_listener.accept()
            print('Got connection from ', conn_listener.last_accepted)
            try:
                while not rospy.is_shutdown() and not self.is_shutdown:
                    # Expect data at >= 50Hz:
                    cmd = array('f', [0,0])
                    self.ready = conn.poll(0.02)
                    if self.ready == False:
                        self.tv.linear.x = 0
                        self.tv.angular.z = 0
                    else:
                        # This trusts that a whole message of the right type was sent...not ideal
                        if conn.recv_bytes_into(cmd) == 8:
                            self.tv.linear.x = cmd[0]
                            self.tv.angular.z = cmd[1]
                        print(cmd[0])
                        print(cmd[1])
            except Exception as e:
                print("Exception in listening thread:")
                print(e)
                self.ready = False
                self.tv.linear.x = 0
                self.tv.angular.z = 0
            conn.close()
        conn_listener.close()

    def sender(self):
        pub = rospy.Publisher('/bluetooth_teleop/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(1000) # 1 kHz joy publish for smooooth operation
        while not rospy.is_shutdown() and not self.is_shutdown:
            if (self.ready):
                pub.publish(self.tv)
            rate.sleep()

if __name__ == '__main__':
    try:
        h = HebiJoySpoofer()
        # This returns, running in a different thread:
        h.startListener()
        # This blocks:
        h.sender()
    except rospy.ROSInterruptException:
        pass
