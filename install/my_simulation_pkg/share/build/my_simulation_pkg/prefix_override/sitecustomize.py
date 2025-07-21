import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sam/MR_Project/install/my_simulation_pkg/share/install/my_simulation_pkg'
