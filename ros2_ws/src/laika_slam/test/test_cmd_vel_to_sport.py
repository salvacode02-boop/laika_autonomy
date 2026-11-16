import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from laika_slam.sport_logic import (remote_keys, remote_event, limit_cmd, goal_key, move_command)

b = [0] * 40
b[2], b[3] = 0x10, 0x04
assert remote_keys(b) == 1040
b[2], b[3] = 0x20, 0x08
assert remote_keys(b) == 2080

assert remote_event(1040, 0) == 'activate' and remote_event(1040, 1040) is None
assert remote_event(2080, 0) == 'stop' and remote_event(2080, 2080) is None
assert remote_event(2080, 1040) == 'stop' and remote_event(16, 0) is None and remote_event(0, 1040) is None
assert remote_event(2080 | 256, 0) == 'stop'             # L2+Y+A también detiene
assert remote_event(2080 | 1040, 0) == 'stop'            # R2+X+L2+Y: detiene, no activa
assert remote_event(2080 | 256, 2080) is None            # sostener L2+Y no repite
assert remote_event(1040 | 256, 0) is None               # activar exige exactamente R2+X
assert remote_event(1040, 2080) == 'activate'            # soltar paro y luego R2+X

assert limit_cmd(1.0, 0.3, 2.0) == (0.25, 0.0, 0.5)
assert limit_cmd(-0.2, 0, 0.3) == (0.0, 0.0, 0.3)  # sin reversa
assert limit_cmd(0.1, 0, -9) == (0.1, 0.0, -0.5)
assert limit_cmd(0, 0, 0) is None and limit_cmd(-0.1, 0, 0) is None
assert limit_cmd(float('nan'), 0, 0) is None and limit_cmd(0.1, 0, float('inf')) is None

c = (0.2, 0.0, 0.1)
assert move_command(10.0, c, 9.9, 9.8, True) == (0.2, 0.0, 0.1)
assert move_command(10.0, c, 9.6, 9.8, True) is None    # cmd viejo
assert move_command(10.0, c, 9.9, 9.4, True) is None    # deadman viejo
assert move_command(10.0, c, 9.9, 9.8, False) is None   # sin aprobar
assert move_command(10.0, c, None, 9.8, True) is None and move_command(10.0, c, 9.9, None, True) is None
assert move_command(10.0, None, 9.9, 9.8, True) is None
assert goal_key(1, 2, 0, 1) == '1.000,2.000,0.000,1.000'
print('ok')
