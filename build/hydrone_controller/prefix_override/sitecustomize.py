import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lh/Documents/joao_pessoa_2026/install/hydrone_controller'
