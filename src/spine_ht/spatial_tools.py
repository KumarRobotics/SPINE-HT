import numpy as np


class SE2Transforms:
    def __init__(self, origin: np.ndarray, rot_radians: float, inverse=False):
        self._origin = origin
        self._rot_mtx = self.create_se2_rotation_matrix(rot_radians)
        self._inverse = inverse

    def transform_pt(self, pt: np.ndarray) -> np.ndarray:
        if self._inverse:
            return self.inverse_transform(pt)

        pt = np.matmul(self._rot_mtx, pt)
        pt += self._origin

        return pt

    def inverse_transform(self, pt: np.ndarray) -> np.ndarray:
        pt -= self._origin
        pt = np.matmul(self._rot_mtx, pt)
        return pt

    def create_se2_rotation_matrix(self, rot_radians):
        cos_theta = np.cos(rot_radians)
        sin_theta = np.sin(rot_radians)

        rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        return rotation_matrix
