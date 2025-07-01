import yaml
import numpy as np


class LidarToImageProjector:

    def __init__(self, K, T, im_w = 640, im_h = 360):
        self.K = K
        self.T = T
        self.im_w = im_w
        self.im_h = im_h


    def project_points_to_image(self, pts):
        # Add homogeneous coordinate
        pts_homo = np.hstack((pts, np.ones((pts.shape[0], 1))))

        # Transform from Lidar to Camera frame
        pts_cam = (self.T @ pts_homo.T).T

        # Remove Lidar points that are behind the camera
        # Otherwise they would be incorrectly projected onto the image
        valid = pts_cam[:, 2] > 1e-6
        pts_cam = pts_cam[valid]

        # Extract depth values (z-coordinate) in the camera coordinate system
        depth = pts_cam[:, 2]

        # Project to image plane
        pts_2d = self.K @ pts_cam[:, :3].T

        # Perspectiv devision: f * x / z, f * y / z
        pixels = (pts_2d[:2] / pts_2d[2]).T

        # Remove pixels that fall outside the image boundaries
        u, v = pixels.T
        mask = (u >= 0) & (u < self.im_w) & (v >= 0) & (v < self.im_h)

        # Keep only valid pixel projections and corresponding depths
        pixels = pixels[mask]

        # Depth in camera coordinates (along the optical axis, i.e., z-direction)
        depth = depth[mask]

        # Filter corresponding original LiDAR points (in vehicle coordinates)
        # x_values: forward direction in vehicle frame
        # y_values: left direction in vehicle frame
        x_values = pts[valid][mask, 0]
        y_values = pts[valid][mask, 1]

        return pixels, depth, x_values, y_values
    
def read_camera_config(path):
    """
    Reads a YAML config file containing camera extrinsic (T) and intrinsic (K) matrices.

    Args:
        path (str): Path to the YAML file.

    Returns:
        K (np.ndarray): 3x3 camera intrinsic matrix.
        T (np.ndarray): 3x4 rigid-body transform matrix (extrinsics).
    """
    with open(path, 'r') as f:
        config = yaml.safe_load(f)

    # Parse intrinsic matrix
    K_data = config['intrinsic_matrix']['data']
    K = np.array(K_data)

    # Parse transform matrix
    T_data = config['transform_matrix']['data']
    T = np.array(T_data)

    return K, T
