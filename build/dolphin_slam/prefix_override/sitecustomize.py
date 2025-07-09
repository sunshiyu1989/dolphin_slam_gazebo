import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cetc3/dolphin_slam_ws/install/dolphin_slam'
