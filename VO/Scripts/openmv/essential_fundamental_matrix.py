import sensor
import image
import time

sensor.reset()
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((320, 240))
sensor.skip_frames(time=2000)
sensor.set_framerate(10)
clock = time.clock()

# Intrinsic matrix (example values; replace with your camera's calibration data)
K = [
    [320, 0, 160],  # fx, skew, cx
    [0, 320, 120],  # skew, fy, cy
    [0, 0, 1]       # homogeneous coordinate
]

KEYPOINTS_SIZE = 8

def transpose(matrix):
    """Transpose a matrix."""
    return [[row[i] for row in matrix] for i in range(len(matrix[0]))]

def multiply_matrices(A, B):
    """Multiply two matrices."""
    return [[sum(a * b for a, b in zip(A_row, B_col)) for B_col in zip(*B)] for A_row in A]

def diagonalize(vector):
    """Create a diagonal matrix from a vector."""
    size = len(vector)
    diag_matrix = [[0] * size for _ in range(size)]
    for i in range(size):
        diag_matrix[i][i] = vector[i]
    return diag_matrix

def normalize_points(points):
    """Normalize points to improve numerical stability."""
    centroid = [sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points)]
    scale = sum(((p[0] - centroid[0])**2 + (p[1] - centroid[1])**2)**0.5 for p in points) / len(points)
    norm_mat = [
        [1 / scale, 0, -centroid[0] / scale],
        [0, 1 / scale, -centroid[1] / scale],
        [0, 0, 1]
    ]
    norm_points = [[(p[0] - centroid[0]) / scale, (p[1] - centroid[1]) / scale] for p in points]
    return norm_points, norm_mat

def svd_manual(A):
    """Simplified SVD for small matrices."""
    AT = transpose(A)
    ATA = multiply_matrices(AT, A)

    # Placeholder eigen decomposition for ATA
    eigvals = [1.0, 0.5, 0.1]  # Example singular values (replace with a real method)
    eigvecs = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # Identity matrix for simplicity

    U = eigvecs
    S = eigvals
    V = eigvecs

    return U, S, V

def estimate_fundamental_matrix(points1, points2):
    """Compute the fundamental matrix using the eight-point algorithm."""
    if len(points1) < 8 or len(points2) < 8:
        print("Not enough points to compute the Fundamental Matrix.")
        return None

    points1, T1 = normalize_points(points1)
    points2, T2 = normalize_points(points2)

    A = []
    for (x1, y1), (x2, y2) in zip(points1, points2):
        A.append([x1 * x2, x1 * y2, x1, y1 * x2, y1 * y2, y1, x2, y2, 1])

    # Compute SVD for matrix A
    U, S, V = svd_manual(A)
    if V is None:
        print("SVD failed to compute.")
        return None
    F = [[V[-1][i] for i in range(3)] for j in range(3)]  # Simplified extraction

    # Enforce rank-2 constraint
    U, S, V = svd_manual(F)
    S = list(S)
    S[-1] = 0  # Force the smallest singular value to zero
    F = multiply_matrices(U, multiply_matrices(diagonalize(S), transpose(V)))

    # Denormalize
    F = multiply_matrices(transpose(T2), multiply_matrices(F, T1))
    return F

def calculate_essential_matrix(F, K):
    """Compute the essential matrix from the fundamental matrix."""
    K_transpose = transpose(K)
    E = multiply_matrices(multiply_matrices(K_transpose, F), K)
    return E

# Main loop for the camera module
frame_width = sensor.width()
frame_height = sensor.height()

last_frame = sensor.snapshot()
lkpoints = last_frame.find_keypoints(max_keypoints=150, threshold=10, scale_factor=1.2)

while True:
    clock.tick()
    current_frame = sensor.snapshot()
    ckpoints = current_frame.find_keypoints(max_keypoints=150, threshold=10, normalized=True)

    if ckpoints and lkpoints:
        matches = image.match_descriptor(lkpoints, ckpoints, threshold=85)

        if matches:
            points1 = []
            points2 = []

            for match in matches.match():
                if 0 <= match[0] < len(lkpoints) and 0 <= match[1] < len(ckpoints):
                    lpoint = (lkpoints[match[0]][0], lkpoints[match[0]][1])
                    cpoint = (ckpoints[match[1]][0], ckpoints[match[1]][1])
                    points1.append(lpoint)
                    points2.append(cpoint)

            if len(points1) >= 8:
                # Compute the fundamental matrix
                F = estimate_fundamental_matrix(points1, points2)
                if F is not None:
                    print("Fundamental Matrix:", F)

                    # Compute the essential matrix
                    E = calculate_essential_matrix(F, K)
                    print("Essential Matrix:", E)

    lkpoints = ckpoints
    print("FPS:", clock.fps())
