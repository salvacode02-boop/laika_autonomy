"""Funciones puras del puente cmd_vel -> sport (sin ROS, para poder probarlas)."""
import math

KEYS_ACTIVATE = 1040  # R2+X (medido en /lowstate)
KEYS_STOP = 2080      # L2+Y
VX_MAX, WZ_MAX = 0.4, 0.5  # 0.4 aprobado por el usuario para medir el umbral de marcha (fase 4, paso 6)
CMD_TIMEOUT, DEADMAN_TIMEOUT, STOP_BURST = 0.3, 0.5, 1.0
LOWSTATE_TIMEOUT = 0.5  # sin /lf/lowstate (20 Hz) no hay botón de paro: se detiene


def remote_keys(wireless_remote):
    return int(wireless_remote[2]) | (int(wireless_remote[3]) << 8)


def _stop_held(keys):
    # Paro por bits: L2+Y detiene aunque haya otros botones presionados.
    return (keys & KEYS_STOP) == KEYS_STOP


def remote_event(keys, prev_keys):
    """Flanco de subida: 'activate', 'stop' o None. El paro gana siempre; L2+Y nunca activa."""
    if _stop_held(keys):
        return None if _stop_held(prev_keys) else 'stop'
    if keys == KEYS_ACTIVATE and prev_keys != KEYS_ACTIVATE:
        return 'activate'
    return None


def limit_cmd(vx, vy, wz):
    """(vx, 0, wz) limitado, o None si hay NaN/inf o el comando queda en cero."""
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        return None
    vx = min(max(vx, 0.0), VX_MAX)  # sin reversa
    wz = min(max(wz, -WZ_MAX), WZ_MAX)
    return None if vx == 0.0 and wz == 0.0 else (vx, 0.0, wz)


def goal_key(x, y, qz, qw):
    """Identificador de meta; aprobar_rutas.py usa la misma formula."""
    return '%.3f,%.3f,%.3f,%.3f' % (x, y, qz, qw)


def move_command(now, cmd, t_cmd, t_deadman, approved):
    """Comando limitado si todo esta en regla; None si hay que parar."""
    if cmd is None or t_cmd is None or t_deadman is None or not approved:
        return None
    if now - t_cmd > CMD_TIMEOUT or now - t_deadman > DEADMAN_TIMEOUT:
        return None
    return limit_cmd(*cmd)
