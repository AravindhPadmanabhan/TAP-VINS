import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse

def transform_trajectory(input_file, output_file):
    # Define rotation: -90 degrees around Z axis
    rotation_base_to_imu_matrix = np.zeros((3, 3))
    rotation_base_to_imu_matrix[0, 1] = 1
    rotation_base_to_imu_matrix[1, 0] = -1
    rotation_base_to_imu_matrix[2, 2] = 1

    trans_base_to_imu = np.array([-0.006, -0.018, -0.058])

    # T_IMU_BASE = [R | t]
    T_IMU_BASE = np.eye(4)
    T_IMU_BASE[:3, :3] = rotation_base_to_imu_matrix
    T_IMU_BASE[:3, 3] = trans_base_to_imu

    with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
        for line in f_in:
            if line.strip() == '' or line.startswith('#'):
                continue  # skip empty lines or comments
            parts = line.strip().split()
            if len(parts) != 8:
                print(f"Skipping invalid line: {line}")
                continue

            timestamp, tx, ty, tz, qx, qy, qz, qw = parts
            translation = np.array([float(tx), float(ty), float(tz)])
            orientation = R.from_quat([float(qx), float(qy), float(qz), float(qw)])

            # Build T_W_IMU = [R | t]
            T_W_IMU = np.eye(4)
            T_W_IMU[:3, :3] = orientation.as_matrix()
            T_W_IMU[:3, 3] = translation

            # Compose: T_W_BASE = T_W_IMU * T_IMU_BASE
            T_W_BASE = T_W_IMU @ T_IMU_BASE

            # Extract new translation and rotation
            translation_new = T_W_BASE[:3, 3]
            rotation_new = R.from_matrix(T_W_BASE[:3, :3])
            q = rotation_new.as_quat()  # (x, y, z, w)

            # Write to output
            f_out.write(f"{timestamp} {translation_new[0]:.6f} {translation_new[1]:.6f} {translation_new[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Transform trajectory from IMU to base frame.')
    parser.add_argument('--input_file', type=str, help='Input trajectory file in TUM format')
    parser.add_argument('--output_file', type=str, help='Output trajectory file in TUM format')
    args = parser.parse_args()
    transform_trajectory(args.input_file, args.output_file)