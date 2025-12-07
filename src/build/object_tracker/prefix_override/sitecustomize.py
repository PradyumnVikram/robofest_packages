import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/azidozide/px4_sitl_ws/src/install/object_tracker'
