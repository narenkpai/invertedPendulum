#!/usr/bin/env python3
"""Compute LQR gains for the inverted pendulum balance sketch.

Model: the stepper is a kinematic actuator, so the control input is cart
acceleration u = x_dotdot directly and pendulum mass drops out. The only
physical parameter is the effective pendulum length l_eff:
  - point mass on a light rod: l_eff = distance pivot -> mass
  - uniform rod of total length L: l_eff = (2/3) * L

State: [x (m), x_dot (m/s), theta (rad, 0 = upright, + = lean toward +x),
        theta_dot (rad/s)]
Control law in the sketch: u = K1*x + K2*x_dot + K3*theta + K4*theta_dot
(the table stores -K of the usual u = -Kx convention, so entries are positive).

Edit Q/R below to retune, run, and paste the printed table into
pendulum_balance.ino.
"""
import numpy as np

G = 9.81
LENGTHS = [0.10, 0.15, 0.20, 0.30, 0.40]

# State weights: [x, x_dot, theta, theta_dot]. Bigger = tighter on that state.
Q = np.diag([60.0, 15.0, 400.0, 40.0])
# Control weight: bigger = gentler accelerations.
R = np.array([[1.0]])


def care(A, B, Q, R):
    """Solve the continuous algebraic Riccati equation via the Hamiltonian."""
    n = A.shape[0]
    Rinv = np.linalg.inv(R)
    H = np.block([[A, -B @ Rinv @ B.T], [-Q, -A.T]])
    w, v = np.linalg.eig(H)
    stable = np.argsort(w.real)[:n]
    U = v[:, stable]
    X = (U[n:, :] @ np.linalg.inv(U[:n, :])).real
    return Rinv @ B.T @ X


print("// Paste into pendulum_balance.ino (u = K1*x + K2*v + K3*th + K4*thd):")
print("const float GAIN_LEN[] = {" + ", ".join(f"{l:.2f}f" for l in LENGTHS) + "};")
rows = []
for l in LENGTHS:
    A = np.array([[0, 1, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 1],
                  [0, 0, G / l, 0]], float)
    B = np.array([[0], [1], [0], [-1 / l]], float)
    K = -care(A, B, Q, R)[0]          # store -K so sketch coefficients are +
    rows.append("  {" + ", ".join(f"{k:.2f}f" for k in K) + "}")
    poles = np.linalg.eigvals(A + B @ K.reshape(1, 4))
    assert all(poles.real < 0), f"unstable design at l={l}"
print("const float GAIN_K[][4] = {")
print(",\n".join(rows))
print("};")
