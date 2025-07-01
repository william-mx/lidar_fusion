import yaml
import numpy as np

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
