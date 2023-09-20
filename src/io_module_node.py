#!/usr/bin/env python3

import rospy
from io_module import IOModule

if __name__ == '__main__':
    rospy.init_node('io_module_node', anonymous=True, disable_signals=True)
    port = rospy.get_param('~port', '/dev/ttyACM0')
    baudrate = rospy.get_param('~baudrate', 9600)
    freq = rospy.get_param('~freq', 10)
    iomodule = IOModule(port, baudrate, freq)
    rospy.on_shutdown(iomodule.shutdown)
    try:
        iomodule.publish_inputs()
    except KeyboardInterrupt:
        iomodule.shutdown()