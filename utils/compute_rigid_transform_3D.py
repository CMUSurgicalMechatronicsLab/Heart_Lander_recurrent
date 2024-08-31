import numpy as np

def compute_rigid_transform(A,B):
    '''
    input: A and B are 3xN matrix of points
    return: R (3x3 rotation matrix), t (3x1 column vector)
    '''

    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # Ensure centroids are 3x1
    centroid_A = centroid_A[:, np.newaxis]
    centroid_B = centroid_B[:, np.newaxis]

    A_sub = np.subtract(A, centroid_A)
    B_sub = np.subtract(B, centroid_B)

    # Multiply both matrices
    H = A_sub @ B_sub.T

    U, S, Vt = np.linalg.svd(H)  # V is returned transposed (Vt)
    V = Vt.T

    # Calculate R
    R = V @ U.T

    # Special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...\n")
        V[:, 2] *= -1
        R = V @ U.T

    # Calculate t
    t = centroid_B - (R @ centroid_A)

    return R, t

if __name__ == '__main__':
    points_A = np.array([[1, 2, 3], [10, 67, 2], [78, 83, 91]]).T
    points_B = np.array([[1, 2, 3], [10, 67, 2], [78, 83, 91]]).T
    R, t = compute_rigid_transform(points_A, points_B)
    print("R:\n", R, "\n")
    print("t:\n", t)