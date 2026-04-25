import numpy as np
import scipy.linalg
import scipy.signal
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ==========================================
# 1. THÔNG SỐ VẬT LÝ
# ==========================================
M = 0.100       
m = 0.04467     
l = 0.200       
J = 5.96e-4     
g = 9.81        
dt = 0.005      

p = J + m * l**2  
det_linear = (M + m) * p - (m * l)**2

# ==========================================
# 2. LQR DESIGN (Dùng cho Catching)
# ==========================================
A = np.zeros((4, 4))
A[0, 1] = 1.0
A[2, 3] = 1.0
A[1, 2] = -(m * l)**2 * g / det_linear
A[3, 2] = (M + m) * m * g * l / det_linear

B = np.zeros((4, 1))
B[1, 0] = p / det_linear
B[3, 0] = -(m * l) / det_linear

Q = np.diag([10.0, 1.0, 100.0, 10.0])
R = np.array([[1.0]])

Ad, Bd, _, _, _ = scipy.signal.cont2discrete((A, B, np.eye(4), np.zeros((4,1))), dt, method='zoh')
P = scipy.linalg.solve_discrete_are(Ad, Bd, Q, R)
K = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)

# ==========================================
# 3. ĐỘNG LỰC HỌC PHI TUYẾN
# ==========================================
def nonlinear_dynamics(state, F):
    x, x_dot, theta, theta_dot = state
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    
    det = (M + m) * p - (m * l * cos_t)**2
    x_ddot = (p * (F + m * l * theta_dot**2 * sin_t) - m * l * cos_t * (m * g * l * sin_t)) / det
    theta_ddot = (-m * l * cos_t * (F + m * l * theta_dot**2 * sin_t) + (M + m) * (m * g * l * sin_t)) / det
    
    return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

def rk4_step(state, F, dt):
    k1 = nonlinear_dynamics(state, F)
    k2 = nonlinear_dynamics(state + 0.5 * dt * k1, F)
    k3 = nonlinear_dynamics(state + 0.5 * dt * k2, F)
    k4 = nonlinear_dynamics(state + dt * k3, F)
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

# ==========================================
# 4. CHẠY MÔ PHỎNG VỚI HYBRID CONTROL
# ==========================================
# Khởi tạo: Con lắc buông thõng xuống đất (pi rad)
state = np.array([0.0, 0.0, math.pi, 0.0])  

history_t, history_x, history_theta, history_F = [], [], [], []
mode_history = [] # Lưu trạng thái bộ điều khiển

for step in range(int(10.0 / dt)):  # Tăng thời gian lên 10 giây
    t = step * dt
    
    # Chuẩn hóa góc theta về đoạn [-pi, pi]
    theta_norm = (state[2] + math.pi) % (2 * math.pi) - math.pi
    
    # HYBRID CONTROLLER LOGIC
    if abs(theta_norm) < 0.3:
        # GIAI ĐOẠN 2: LQR CATCHING (Góc lệch nhỏ hơn ~17 độ)
        state_lqr = np.array([state[0], state[1], theta_norm, state[3]])
        F = float(-(K @ state_lqr.reshape(4, 1))[0, 0])
        mode_history.append(1) # 1 là LQR
    else:
        # GIAI ĐOẠN 1: ENERGY SHAPING SWING-UP
        # Tính năng lượng hiện tại (E = 0 tại vị trí thẳng đứng đứng yên)
        E = 0.5 * p * state[3]**2 + m * g * l * (math.cos(state[2]) - 1.0)
        
        # Bơm năng lượng tỷ lệ thuận với độ hụt năng lượng và chiều chuyển động
        # Hệ số 50.0 là tham số tuning (Gain)
        F_swing = 50.0 * E * math.cos(state[2]) * state[3]
        
        # Thêm hàm phạt vị trí nhẹ để xe không chạy vô tận ra khỏi màn hình
        F_pos_correction = -2.0 * state[0] - 1.0 * state[1]
        
        # Giới hạn lực đẩy thực tế của động cơ (Saturation)
        F = np.clip(F_swing + F_pos_correction, -15.0, 15.0)
        mode_history.append(0) # 0 là Swing-up

    history_t.append(t)
    history_x.append(state[0])
    history_theta.append(state[2])
    history_F.append(F)
    
    state = rk4_step(state, F, dt)

# ==========================================
# 5. VẼ ĐỒ THỊ VÀ ANIMATION
# ==========================================
fig_static, axs = plt.subplots(3, 1, figsize=(10, 8))
fig_static.canvas.manager.set_window_title('Swing-Up & LQR Transient')

axs[0].plot(history_t, history_x, 'b-', lw=2)
axs[0].set_ylabel('Position (m)')
axs[0].grid(True, linestyle='--', alpha=0.7)

# Biểu diễn góc bằng độ cho dễ hình dung quá trình swing-up
theta_deg = [math.degrees(th) for th in history_theta]
axs[1].plot(history_t, theta_deg, 'r-', lw=2)
axs[1].axhline(0, color='black', lw=1, ls='--')
axs[1].axhline(180, color='gray', lw=1, ls=':', label='Downwards')
axs[1].set_ylabel('Angle (Degree)')
axs[1].grid(True, linestyle='--', alpha=0.7)
axs[1].legend(loc='upper right')

# Tô màu nền để phân biệt chế độ điều khiển
for i in range(len(history_t) - 1):
    if mode_history[i] == 1:
        axs[2].axvspan(history_t[i], history_t[i+1], color='lightgreen', alpha=0.3, lw=0)

axs[2].plot(history_t, history_F, 'g-', lw=2)
axs[2].set_ylabel('Control Effort (N)')
axs[2].set_xlabel('Time (s)')
axs[2].grid(True, linestyle='--', alpha=0.7)
axs[2].text(0.5, 0.9, 'White background: Swing-Up Phase\nGreen background: LQR Catching', 
            transform=axs[2].transAxes, bbox=dict(facecolor='white', alpha=0.8))

plt.tight_layout()

# --- ANIMATION ---
fig_ani, ax_ani = plt.subplots(figsize=(8, 5))
fig_ani.canvas.manager.set_window_title('Swing-Up Animation')
ax_ani.set_xlim(-1.0, 1.0)
ax_ani.set_ylim(-0.6, 0.6)
ax_ani.set_aspect('equal')
ax_ani.grid(True, alpha=0.3)
ax_ani.axhline(0, color='black', lw=2)

cart_w, cart_h = 0.08, 0.04
cart_patch = plt.Rectangle((0, 0), cart_w, cart_h, fc='#3498db', ec='black')
ax_ani.add_patch(cart_patch)
rod_line, = ax_ani.plot([], [], lw=4, color='#e74c3c')
title_text = ax_ani.text(-0.9, 0.5, '', fontsize=12, weight='bold')

def init():
    return cart_patch, rod_line, title_text

# Cứ mỗi frame nhảy 4 step để animation chạy thực tế hơn (tăng tốc độ hiển thị)
frame_skip = 4 
def update(frame):
    idx = frame * frame_skip
    if idx >= len(history_t): idx = len(history_t) - 1
    
    x = history_x[idx]
    theta = history_theta[idx]
    
    cx, cy = x, 0.0
    cart_patch.set_xy((cx - cart_w/2, cy - cart_h/2))
    
    rod_len = 2 * l
    px = cx + rod_len * math.sin(theta)
    py = cy + rod_len * math.cos(theta)
    
    rod_line.set_data([cx, px], [cy, py])
    
    mode = "LQR Catching" if mode_history[idx] == 1 else "Energy Swing-up"
    title_text.set_text(f"Mode: {mode}\nTime: {history_t[idx]:.2f}s")
    
    return cart_patch, rod_line, title_text

ani = FuncAnimation(fig_ani, update, frames=len(history_t)//frame_skip, init_func=init, interval=dt*1000*frame_skip, blit=True)

plt.show()