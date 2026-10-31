import numpy as np


class SmoothJointRamp:
    def __init__(self, joint_indices, duration_s):
        self.joint_indices = np.array(list(joint_indices), dtype=np.int64)
        self.duration_s = float(duration_s)
        self.active = False
        self._start_q = None
        self._target_q = None
        self._start_time_s = 0.0

    def start(self, start_q, target_q, now_s):
        self._start_q = np.array(start_q, dtype=np.float32, copy=True)
        self._target_q = np.array(target_q, dtype=np.float32, copy=True)
        self._start_time_s = float(now_s)
        self.active = True

    def cancel(self):
        self.active = False

    def sample(self, now_s):
        if self._start_q is None or self._target_q is None:
            raise RuntimeError("SmoothJointRamp.sample() called before start().")

        elapsed_s = max(0.0, float(now_s) - self._start_time_s)
        phase = 1.0 if self.duration_s <= 0.0 else min(elapsed_s / self.duration_s, 1.0)
        alpha = phase * phase * (3.0 - 2.0 * phase)

        q_des = np.array(self._start_q, dtype=np.float32, copy=True)
        start = self._start_q[self.joint_indices]
        target = self._target_q[self.joint_indices]
        q_des[self.joint_indices] = start + alpha * (target - start)

        done = phase >= 1.0
        self.active = not done
        return q_des, done
