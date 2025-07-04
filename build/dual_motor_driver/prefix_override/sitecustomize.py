import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dyllon/Documents/SlamRobot/install/dual_motor_driver'
