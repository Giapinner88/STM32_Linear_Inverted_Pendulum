from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import mujoco
import mujoco.viewer
import numpy as np
from scipy.linalg import solve_continuous_are

@dataclass
class Params:
    M: float = 0.26         
    m: float = 0.04467      
    l: float = 0.1          
    J: float = 5.96e-4      
    g: float = 9.81         
    dt: float = 0.005       
    F_max: float = 10.0     

    switch_angle: float = 0.5   

    # Giảm mạnh lực giữ xe, tăng nhẹ lực bơm năng lượng
    k_E: float = 20.0    
    k_x: float = 0.3     
    k_v: float = 0.5     

    # Hệ số hướng tương tác: Thay đổi giữa 1.0 hoặc -1.0 tùy thuộc vào trục XML
    coupling_sign: float = 1.0  

    Q: np.ndarray = field(default_factory=lambda: np.diag([10.0, 1.0, 100.0, 10.0]))
    R: np.ndarray = field(default_factory=lambda: np.array([[0.1]]))

    K_lqr: Optional[np.ndarray] = None

    def __post_init__(self):
        self.K_lqr = compute_lqr_gain(self)

def compute_lqr_gain(p: Params) -> np.ndarray:
    M, m, l, J, g, s = p.M, p.m, p.l, p.J, p.g, p.coupling_sign
    
    delta = J * (M + m) - (m * l)**2
    
    A = np.array([
        [0, 1, 0, 0],
        [0, 0, -s * (m * l)**2 * g / delta, 0],
        [0, 0, 0, 1],
        [0, 0, s * ((M + m) * m * g * l) / delta, 0],
    ])
    
    B = np.array([
        [0],
        [J / delta],
        [0],
        [-s * (m * l) / delta],
    ])
    
    P = solve_continuous_are(A, B, p.Q, p.R)
    K = np.linalg.inv(p.R) @ B.T @ P
    print(f"[LQR] Gain K = {K.flatten()}")
    return K

def read_state(data: mujoco.MjData) -> tuple[float, float, float, float]:
    return (
        float(data.sensordata[0]),
        float(data.sensordata[1]),
        float(data.sensordata[2]),
        float(data.sensordata[3])
    )

def energy_shaping(state: tuple, p: Params) -> float:
    x, x_dot, theta, th_dot = state
    m, l, J, g = p.m, p.l, p.J, p.g

    E = 0.5 * J * th_dot**2 - m * g * l * (1.0 + np.cos(theta))
    
    # Tích hợp coupling_sign vào Energy Shaping để đồng bộ hướng đẩy
    u_E = -p.coupling_sign * p.k_E * E * np.cos(theta) * th_dot
    u_cart = -p.k_x * x - p.k_v * x_dot

    u = u_E + u_cart
    return float(np.clip(u, -p.F_max, p.F_max))

def lqr_control(state: tuple, p: Params) -> float:
    x, x_dot, theta, th_dot = state
    theta_norm = (theta - np.pi + np.pi) % (2 * np.pi) - np.pi
    
    err = np.array([x, x_dot, theta_norm, th_dot])
    u_raw = float(-(p.K_lqr @ err.reshape(4, 1))[0, 0])
    u = float(np.clip(u_raw, -p.F_max, p.F_max))
    
    # DEBUG: in ra nếu góc gần upright
    if abs(theta_norm) < 0.05:
        print(f"[DEBUG LQR] theta={np.degrees(theta):.1f}° | theta_norm={np.degrees(theta_norm):.1f}° | K={p.K_lqr.flatten()} | err={err} | u_raw={u_raw:.3f} | u_clipped={u:.3f}")
    
    return u

def run_simulation(xml_path: str = "cartpole.xml") -> None:
    p = Params()
    model = mujoco.MjModel.from_xml_path(xml_path)
    data  = mujoco.MjData(model)

    data.qpos[0] = 0.0
    data.qpos[1] = 0.01  # Start from hanging for swing-up test
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    t_last_print = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        time.sleep(2)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_side")

        while viewer.is_running():
            step_start = time.time()
            state = read_state(data)
            _, _, theta, _ = state

            angle_from_upright = abs((theta - np.pi + np.pi) % (2 * np.pi) - np.pi)

            if angle_from_upright < p.switch_angle:
                mode = "LQR"
                u = lqr_control(state, p)
            else:
                mode = "SWING"
                u = energy_shaping(state, p)

            data.ctrl[0] = u
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))

            if data.time - t_last_print >= 0.5:
                x, xd, th, thd = state
                th_err = (th - np.pi + np.pi) % (2 * np.pi) - np.pi
                print(f"t={data.time:6.2f}s | mode={mode:5s} | x={x:+.3f} | θ-π={np.degrees(th_err):+7.2f}° | u={u:+.2f} N")
                t_last_print = data.time

if __name__ == "__main__":
    import os
    import sys
    
    xml_path = os.path.join(os.path.dirname(__file__), "cartpole.xml")
    if not os.path.isfile(xml_path):
        print(f"[ERROR] Không tìm thấy file XML: {xml_path}")
        sys.exit(1)
        
    run_simulation(xml_path)