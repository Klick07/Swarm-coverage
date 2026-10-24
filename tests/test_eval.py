import numpy as np
from scipy.spatial.transform import Rotation

from eval.cslam_eval import ate, decode_key


def test_decode_labeled_symbol():
    key = (ord("g") << 56) | (ord("B") << 48) | 1234  # gtsam::LabeledSymbol('g', 'A'+1, 1234)
    assert decode_key(key) == (1, 1234)


def test_ate_is_zero_after_rigid_motion():
    rng = np.random.default_rng(0)
    ref = rng.uniform(-5, 5, (100, 3))
    R = Rotation.from_euler("xyz", [0.3, -0.2, 1.1]).as_matrix()
    est = ref @ R.T + np.array([2.0, -1.0, 0.5])
    assert ate(est, ref)[0] < 1e-9


def test_ate_measures_noise():
    rng = np.random.default_rng(1)
    ref = rng.uniform(-5, 5, (2000, 3))
    est = ref + rng.normal(0, 0.1, ref.shape)
    assert abs(ate(est, ref)[0] - 0.1 * np.sqrt(3)) < 0.02
