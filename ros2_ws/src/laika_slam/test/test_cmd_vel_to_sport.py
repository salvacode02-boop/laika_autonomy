import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from laika_slam.sport_logic import (remote_keys, remote_event, limit_cmd, goal_key, move_command, SinCable, ruta_vmax)

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

assert limit_cmd(1.0, 0.3, 2.0) == (0.3, 0.0, 0.5)
assert limit_cmd(-0.2, 0, 0.3) == (0.0, 0.0, 0.4)  # sin reversa; giro en el lugar subido a 0.4
assert limit_cmd(0.1, 0, -9) == (0.25, 0.0, -0.5)  # vx subido al mínimo de marcha
assert limit_cmd(0.1, 0, 0.1) == (0.25, 0.0, 0.1)  # caminando, el giro pequeño se respeta
assert limit_cmd(0.01, 0, 0.03) is None            # ruido: no se sube al mínimo
assert limit_cmd(0, 0, -0.2) == (0.0, 0.0, -0.4)
assert limit_cmd(0, 0, 0) is None and limit_cmd(-0.1, 0, 0) is None
# velocidad de ruta: escala la salida de Nav2 (tope 0.3) a la elegida, acotada a [0.25, 0.5]
assert abs(limit_cmd(0.3, 0, 0, vmax=0.45)[0] - 0.45) < 1e-9
assert limit_cmd(1.0, 0, 0, vmax=0.9)[0] == 0.5
assert limit_cmd(0.15, 0, 0, vmax=0.45)[0] == 0.25        # 0.225 -> mínimo de marcha
assert limit_cmd(0.3, 0, 0, vmax=0.1)[0] == 0.25          # más lento que 0.25 no camina
assert ruta_vmax(float('nan')) == 0.3
assert move_command(10.0, (0.3, 0.0, 0.0), 9.9, 9.8, True, vmax=0.4)[0] == 0.4
assert limit_cmd(float('nan'), 0, 0) is None and limit_cmd(0.1, 0, float('inf')) is None

c = (0.2, 0.0, 0.1)
assert move_command(10.0, c, 9.9, 9.8, True) == (0.25, 0.0, 0.1)
assert move_command(10.0, c, 9.6, 9.8, True) is None    # cmd viejo
assert move_command(10.0, c, 9.9, 9.4, True) is None    # deadman viejo
assert move_command(10.0, c, 9.9, 9.8, False) is None   # sin aprobar
assert move_command(10.0, c, None, 9.8, True) is None and move_command(10.0, c, 9.9, None, True) is None
assert move_command(10.0, None, 9.9, 9.8, True) is None
assert goal_key(1, 2, 0, 1) == '1.000,2.000,0.000,1.000'

# --- modo sin cable ---
def aprobada(sc, k='A'):
    sc.on_goal(k); sc.on_approved(k)
    return sc

sc = SinCable(); sc.on_goal('A'); sc.on_approved('')            # el '' periodico se ignora
assert sc.state == 'IDLE'
assert sc.on_event('activate', 0) is None and sc.state == 'IDLE'  # R2+X antes del Enter no mueve
sc.on_approved('A'); assert sc.state == 'APROBADA'                # Enter sin R2+X: no mueve
assert sc.tick(1000) is None and sc.state == 'APROBADA'
assert sc.on_event('activate', 5) == 'run' and sc.state == 'RUN'
assert sc.on_goal('A') is None and sc.state == 'RUN'              # eco de la republicacion
assert sc.on_event('activate', 6) is None and sc.state == 'RUN'

sc = aprobada(SinCable())                                         # L2+Y y luego R2+X no mueve
assert sc.on_event('stop', 0) == 'stop' and sc.state == 'IDLE'
sc.on_approved('A'); assert sc.state == 'IDLE'                    # reaprobacion de la misma clave ignorada
assert sc.on_event('activate', 1) is None
assert SinCable().on_event('stop', 0) == 'stop'                   # L2+Y en IDLE tambien manda StopMove

sc = aprobada(SinCable()); sc.on_event('activate', 0)             # meta nueva en RUN para
assert sc.on_goal('B') == 'stop' and sc.state == 'IDLE'
sc.on_approved('A'); assert sc.state == 'IDLE'                    # aprobacion vieja no vale para B
sc.on_approved('B'); assert sc.state == 'APROBADA'

sc = aprobada(SinCable()); sc.on_status({'g0': 6})                # goal viejo abortado antes del R2+X
assert sc.on_event('activate', 0) == 'run'
assert sc.on_status({'g0': 6, 'g1': 6}) == 'stop'                 # el nuevo (llego/aborto)... g1 no estaba
sc = aprobada(SinCable()); sc.on_status({'g0': 2}); sc.on_event('activate', 0)
assert sc.on_status({'g0': 6}) is None and sc.state == 'RUN'      # el viejo no cuenta
assert sc.on_status({'g0': 6, 'g1': 2}) is None
assert sc.on_status({'g0': 6, 'g1': 4}) == 'stop' and sc.state == 'IDLE'   # llego (4)
assert sc.on_event('activate', 1) is None and sc.on_event('activate', 2) is None  # otro R2+X no mueve
sc.on_approved('A'); assert sc.state == 'IDLE'

sc = aprobada(SinCable()); sc.on_event('activate', 100)           # timeout de 90 s
assert sc.tick(189.0) is None and sc.tick(190.5) == 'stop' and sc.state == 'IDLE'
sc = aprobada(SinCable()); sc.on_event('activate', 0)
assert sc.end() == 'stop' and sc.consumed                         # watchdog

# reinicio de sesion: solo detiene, nunca arranca
sc = aprobada(SinCable())                                         # aprobada + sesion nueva: R2+X no entra a RUN
assert sc.on_session_reset() == 'stop' and sc.state == 'IDLE'
assert sc.on_event('activate', 0) is None and sc.state == 'IDLE'
sc.on_approved('A'); assert sc.state == 'IDLE'                    # la misma clave no se reaprueba
sc.on_goal('A'); sc.on_approved('A'); assert sc.state == 'APROBADA'  # meta nueva con el mismo valor si vale
sc = aprobada(SinCable()); sc.on_event('activate', 0)             # en RUN: StopMove
assert sc.on_session_reset() == 'stop' and sc.state == 'IDLE' and sc.goal is None
assert SinCable().on_session_reset() is None                      # en IDLE no manda nada
print('ok')
