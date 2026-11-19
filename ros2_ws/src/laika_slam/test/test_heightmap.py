import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from laika_slam.heightmap_node import HeightGrid


def test_heightmap():
    global rng, pose
    rng = np.random.RandomState(0)
    pose = (0.0, 0.0, 0.3, 0.0)


    def scene(n=60000):
        """Suelo z=0 con ruido de 1 cm, escalón de 10 cm en x>1.5 y un techo a 1.0 m en x<-1.5."""
        xy = rng.uniform(-3, 3, (n, 2))
        z = rng.normal(0, 0.01, n) + np.where(xy[:, 0] > 1.5, 0.10, 0.0)
        return np.column_stack([xy, z]), xy


    g = HeightGrid()
    for k in range(5):
        p, xy = scene()
        roof = np.column_stack([rng.uniform(-3, -1.5, (3000, 2)) * [1, 1], np.full(3000, 1.0)])
        g.update(np.concatenate([p, roof]), pose, 0.2 * k)
    c = g.cloud(0.8)
    x, y, gz, top, over = c.T
    plane = (np.hypot(x, y) > 0.6) & (x < 1.3) & (x > -1.3) & (np.abs(y) < 2.5)
    assert plane.sum() > 1000 and np.sqrt(np.mean(gz[plane] ** 2)) < 0.015, np.sqrt(np.mean(gz[plane] ** 2))
    hi, lo = gz[(x > 1.7) & (x < 2.8)].mean(), gz[(x > 0.5) & (x < 1.3)].mean()
    assert abs((hi - lo) - 0.10) < 0.02, hi - lo
    assert np.isnan(over[(x > 0) & (x < 1)]).all() and not np.isnan(over[x < -1.7]).all()
    assert abs(np.nanmean(over[x < -1.7]) - 1.0) < 0.02
    # huella del robot descartada
    assert not ((np.abs(x) < 0.3) & (np.abs(y) < 0.15)).any()
    # memoria de 2 s: tras 3 s sin ver el techo, overhead expira; el suelo sigue
    assert np.isnan(g.cloud(5.0)[:, 4]).all() and len(g.cloud(5.0)) == len(c)
    # suelo caduco (>60 s) se reinicia en vez de promediar
    g2 = HeightGrid()
    flat = np.column_stack([rng.uniform(1, 2, (3000, 2)), np.zeros(3000)])
    g2.update(flat, pose, 0.0)
    g2.update(flat + [0, 0, 0.5], pose, 100.0)
    assert abs(np.nanmean(g2.cloud(100.0)[:, 2]) - 0.5) < 0.02
    # recentrado: al alejarse 3 m, las celdas viejas se conservan en su sitio global
    g.update(scene()[0] + [3, 0, 0], (3.0, 0.0, 0.3, 0.0), 1.0)
    c2 = g.cloud(1.0)
    old = (c2[:, 0] > -0.9) & (c2[:, 0] < -0.2) & (np.abs(c2[:, 1]) < 2)
    assert old.sum() > 100 and abs(np.nanmean(c2[old, 2])) < 0.01
    assert c2[:, 0].min() < -0.9 and c2[:, 0].max() > 5.0
    pass

