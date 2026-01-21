import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/venkatesh/gesture-controlled-dobot-arm/install/gesture_control_pkg'
