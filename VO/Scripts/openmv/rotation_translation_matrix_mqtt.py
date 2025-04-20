import sensor, image, time, math
import network
from mqtt import MQTTClient
import json

################################################################################
# Basic 2D/3D matrix utilities (same as your fundamental/essential matrix code)
################################################################################

def mat_mul(A, B):
    rA = len(A)
    cA = len(A[0])
    rB = len(B)
    cB = len(B[0])
    result = [[0.0 for _ in range(cB)] for _ in range(rA)]
    for i in range(rA):
        for j in range(cB):
            s = 0.0
            for k in range(cA):
                s += A[i][k]*B[k][j]
            result[i][j] = s
    return result

def mat_transpose(M):
    r = len(M)
    c = len(M[0])
    MT = [[0.0 for _ in range(r)] for _ in range(c)]
    for i in range(r):
        for j in range(c):
            MT[j][i] = M[i][j]
    return MT

def mat_det_3x3(M):
    return (M[0][0]*M[1][1]*M[2][2]
          + M[0][1]*M[1][2]*M[2][0]
          + M[0][2]*M[1][0]*M[2][1]
          - M[0][2]*M[1][1]*M[2][0]
          - M[0][1]*M[1][0]*M[2][2]
          - M[0][0]*M[1][2]*M[2][1])

def diag_matrix_3x3(s):
    return [
        [s[0], 0.0,   0.0 ],
        [0.0,  s[1],  0.0 ],
        [0.0,  0.0,   s[2]]
    ]

################################################################################
# Jacobi for 3x3 or NxN ...
################################################################################

def jacobi_eig_sym(M, n, max_sweeps=30, epsilon=1e-12):
    A = [[float(M[i][j]) for j in range(n)] for i in range(n)]
    V = [[0.0 for _ in range(n)] for _ in range(n)]
    for i in range(n):
        V[i][i] = 1.0

    def max_offdiag(A):
        p, q = 0, 1
        max_val = 0.0
        for i in range(n):
            for j in range(i+1, n):
                if abs(A[i][j]) > max_val:
                    max_val = abs(A[i][j])
                    p, q = i, j
        return p, q, max_val

    for _sweep in range(max_sweeps):
        p, q, max_val = max_offdiag(A)
        if max_val < epsilon:
            break
        alpha = (A[q][q] - A[p][p]) / (2.0*A[p][q])
        t = math.copysign(1.0, alpha) / (abs(alpha) + math.sqrt(1.0+alpha*alpha))
        c = 1.0/math.sqrt(1.0+t*t)
        s = t*c
        tau = s/(1.0+c)
        a_pp = A[p][p]
        a_qq = A[q][q]
        a_pq = A[p][q]
        A[p][p] = a_pp - t*a_pq
        A[q][q] = a_qq + t*a_pq
        A[p][q] = 0.0
        A[q][p] = 0.0
        for i in range(n):
            if i != p and i != q:
                a_ip = A[i][p]
                a_iq = A[i][q]
                A[i][p] = a_ip - s*a_iq
                A[p][i] = A[i][p]
                A[i][q] = a_iq + s*a_ip
                A[q][i] = A[i][q]
        for i in range(n):
            v_ip = V[i][p]
            v_iq = V[i][q]
            V[i][p] = v_ip - s*v_iq
            V[i][q] = v_iq + s*v_ip

    eigvals = [A[i][i] for i in range(n)]
    idxs = list(range(n))
    idxs.sort(key=lambda i: eigvals[i], reverse=False)
    eigvals_sorted = [eigvals[i] for i in idxs]
    V_sorted = [[0.0 for _ in range(n)] for _ in range(n)]
    for new_col, old_i in enumerate(idxs):
        for row in range(n):
            V_sorted[row][new_col] = V[row][old_i]
    return eigvals_sorted, V_sorted

################################################################################
# Solve for fundamental matrix (8-point) ...
################################################################################

def solve_fundamental_via_eig(A):
    N = len(A)
    B = [[0.0]*9 for _ in range(9)]
    for i in range(N):
        row_i = A[i]
        for r in range(9):
            for c in range(9):
                B[r][c] += row_i[r]*row_i[c]
    eigvals, eigvecs = jacobi_eig_sym(B, 9, max_sweeps=50, epsilon=1e-12)
    f = [eigvecs[row][0] for row in range(9)]
    return f

def normalize_points(pts):
    cx = sum(p[0] for p in pts)/len(pts)
    cy = sum(p[1] for p in pts)/len(pts)
    shifted = [(p[0]-cx, p[1]-cy) for p in pts]
    avg_dist = 0.0
    for (x,y) in shifted:
        avg_dist += math.sqrt(x*x + y*y)
    avg_dist /= len(pts)
    scale = math.sqrt(2)/avg_dist if avg_dist > 1e-12 else 1.0
    T = [
        [scale, 0.0,   -scale*cx],
        [0.0,   scale, -scale*cy],
        [0.0,   0.0,    1.0     ]
    ]
    new_pts = []
    for (x,y) in shifted:
        x2 = scale*x
        y2 = scale*y
        new_pts.append((x2, y2))
    return new_pts, T

def compute_fundamental_matrix_8_point(pts1, pts2):
    pts1_norm, T1 = normalize_points(pts1)
    pts2_norm, T2 = normalize_points(pts2)
    A = []
    for (x1, y1), (x2, y2) in zip(pts1_norm, pts2_norm):
        A.append([x1*x2, x1*y2, x1,
                  y1*x2, y1*y2, y1,
                  x2,    y2,    1.0])
    f = solve_fundamental_via_eig(A)
    F_approx = [
        [f[0], f[1], f[2]],
        [f[3], f[4], f[5]],
        [f[6], f[7], f[8]]
    ]
    U3, S3, V3t = svd_3x3(F_approx)
    S3[2] = 0.0
    F_rank2 = mat_mul(mat_mul(U3, diag_matrix_3x3(S3)), V3t)
    T2t = mat_transpose(T2)
    temp = mat_mul(T2t, F_rank2)
    F_final = mat_mul(temp, T1)
    return F_final

def svd_3x3(M):
    MT = mat_transpose(M)
    B = mat_mul(MT, M)
    eigvalsB, Vtemp = jacobi_eig_sym(B, 3, max_sweeps=20, epsilon=1e-12)
    indices = [0,1,2]
    indices.sort(key=lambda i: eigvalsB[i], reverse=True)
    svals = [0.0, 0.0, 0.0]
    Vmat = [[0.0]*3 for _ in range(3)]
    for new_i, old_i in enumerate(indices):
        svals[new_i] = math.sqrt(max(eigvalsB[old_i], 0.0))
        for row in range(3):
            Vmat[row][new_i] = Vtemp[row][old_i]
    Vt = mat_transpose(Vmat)
    Sinv = [[0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]]
    for i in range(3):
        if abs(svals[i]) > 1e-12:
            Sinv[i][i] = 1.0/svals[i]
    MV = mat_mul(M, Vmat)
    # We do a second approach to get U orthonormal: B2 = M M^T
    B2 = mat_mul(M, MT)
    eigvalsB2, Utemp2 = jacobi_eig_sym(B2, 3, max_sweeps=20, epsilon=1e-12)
    indices2 = [0,1,2]
    indices2.sort(key=lambda i: eigvalsB2[i], reverse=True)
    Umat = [[0.0]*3 for _ in range(3)]
    for new_i, old_i in enumerate(indices2):
        for row in range(3):
            Umat[row][new_i] = Utemp2[row][old_i]
    if mat_det_3x3(Umat) < 0.0:
        for row in range(3):
            Umat[row][2] *= -1.0
    if mat_det_3x3(Vmat) < 0.0:
        for row in range(3):
            Vmat[row][2] *= -1.0
        Vt = mat_transpose(Vmat)
    return Umat, svals, Vt

def decompose_essential_matrix(E):
    U, S, Vt = svd_3x3(E)
    S[0] = 1.0
    S[1] = 1.0
    S[2] = 0.0
    E_fixed = mat_mul(mat_mul(U, diag_matrix_3x3(S)), Vt)
    U, _, Vt = svd_3x3(E_fixed)
    Rtemp = mat_mul(U, Vt)
    if mat_det_3x3(Rtemp) < 0:
        for i in range(3):
            U[i][2] *= -1.0
    W = [
        [0.0, -1.0, 0.0],
        [1.0,  0.0, 0.0],
        [0.0,  0.0, 1.0]
    ]
    Wt = mat_transpose(W)
    R1 = mat_mul(mat_mul(U, W), Vt)
    R2 = mat_mul(mat_mul(U, Wt), Vt)
    T = [U[0][2], U[1][2], U[2][2]]
    return (R1, R2, T)

################################################################################
# MQTT / Network Setup
################################################################################

# Adjust these to your actual Wi-Fi credentials
SSID = "ZTE_OZ"
KEY = "26058617"

# Adjust the following to match your broker settings
broker = "broker.emqx.io"
port = 1883
client_id = "nicla_vision_rotation_translation"
user = "emqx"        # or None if not needed
password = "public"  # or None if not needed
topic = "NABD/Vision"
qos = 0

print("Connecting to WiFi...")

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, KEY)

while not wlan.isconnected():
    print('Trying to connect to "{:s}"'.format(SSID))
    time.sleep_ms(1000)

print("WiFi connected:", wlan.ifconfig())

client = MQTTClient(client_id, broker, port=port, user=user, password=password)
try:
    client.connect()
    print("Connected to MQTT Broker.")
except Exception as e:
    print("Failed to connect to MQTT Broker:", e)

################################################################################
# OpenMV Sensor Setup
################################################################################

sensor.reset()
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((320, 240))
sensor.set_framerate(10)
sensor.skip_frames(time=2000)
clock = time.clock()

# Example Intrinsic camera matrix
K = [
    [434.7306,  0.0000, 143.5281],
    [  0.0000, 433.5853, 115.9673],
    [  0.0000,   0.0000,   1.0000]
]


max_keypoints   = 100
threshold       = 20
scale_factor    = 1.2
match_threshold = 100

last_frame = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
lkpoints   = None

def mat_print(label, M):
    print(label)
    for row in M:
        print("   ", ["{:.3f}".format(v) for v in row])

################################################################################
# Main Loop
################################################################################

while True:
    clock.tick()
    current_frame = sensor.snapshot()
    ckpoints = current_frame.find_keypoints(max_keypoints=max_keypoints,
                                            threshold=threshold,
                                            scale_factor=scale_factor)

    if lkpoints and ckpoints:
        matches = image.match_descriptor(lkpoints, ckpoints, threshold=match_threshold)
        if matches:
            pts1 = []
            pts2 = []
            for m in matches.match():
                x1, y1 = lkpoints[m[0]][0], lkpoints[m[0]][1]
                x2, y2 = ckpoints[m[1]][0], ckpoints[m[1]][1]
                pts1.append((x1, y1))
                pts2.append((x2, y2))

            if len(pts1) >= 8:
                # 1) Fundamental matrix
                F = compute_fundamental_matrix_8_point(pts1, pts2)

                # 2) Essential = K^T F K
                Kt = mat_transpose(K)
                E_temp = mat_mul(mat_mul(Kt, F), K)

                # 3) Decompose E
                R1, R2, T = decompose_essential_matrix(E_temp)

                mat_print("F (Fundamental):", F)
                mat_print("E (Essential):", E_temp)
                mat_print("R1:", R1)
                mat_print("R2:", R2)
                print("T:", ["{:.3f}".format(x) for x in T])
                print("-----------------")

                data = {
                    "R1": R1,  # 3x3
                    "R2": R2,  # 3x3
                    "T": T     # 3-vector
                }

                # Convert data to JSON; be mindful that 2D lists become nested arrays
                message = json.dumps(data)
                # Publish
                client.publish(topic, message, qos=qos)
                print("Published rotation/translation to MQTT.")

    last_frame.replace(current_frame)
    lkpoints = ckpoints

    # Display FPS
    current_frame.draw_string(2, 2, "FPS: %.2f" % clock.fps(), color=(255))
