import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/marlboro/ws_diff_drive/install/diff_drive_fusion'
