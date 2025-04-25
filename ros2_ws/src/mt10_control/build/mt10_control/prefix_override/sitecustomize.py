import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mt/ros2_ws/src/mt10_control/install/mt10_control'
