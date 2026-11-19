import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from laika_slam.session_manager import run_reset


class Fake:
    def __init__(self, fail=()):
        self.calls, self.fail = [], set(fail)

    def __getattr__(self, name):
        def f():
            self.calls.append(name)
            return (name not in self.fail), ''
        return f


ORDER = ['check_idle', 'cancel_goals', 'save_map', 'stop_slam', 'reset_odom', 'clear_state',
         'bump_session', 'start_slam']


def test_order_all_pass():
    io = Fake()
    ok, lines = run_reset(io)
    assert ok and io.calls == ORDER and [l[:6] for l in lines] == ['PASS %d' % i for i in range(1, 9)]


def test_not_idle_aborts_untouched():
    io = Fake(fail={'check_idle'})
    ok, lines = run_reset(io)
    assert not ok and io.calls == ['check_idle'] and lines[0].startswith('FAIL 1')


def test_noncritical_fail_continues():
    io = Fake(fail={'save_map', 'clear_state'})
    ok, lines = run_reset(io)
    assert ok and io.calls == ORDER and lines[2].startswith('FAIL 3')


def test_odom_fail_still_restarts_slam():
    io = Fake(fail={'reset_odom'})
    ok, _ = run_reset(io)
    assert not ok and io.calls == ORDER[:5] + ['start_slam']  # sin bump de sesion, pero slam vuelve
