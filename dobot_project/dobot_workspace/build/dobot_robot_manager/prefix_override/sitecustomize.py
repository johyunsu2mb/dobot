import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kiro06/Documents/dobot_project/dobot_workspace/install/dobot_robot_manager'
