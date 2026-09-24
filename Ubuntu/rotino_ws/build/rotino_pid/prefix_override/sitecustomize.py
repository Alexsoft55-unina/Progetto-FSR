import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aldo/Scrivania/FSR/Progetto-FSR/Ubuntu/rotino_ws/install/rotino_pid'
