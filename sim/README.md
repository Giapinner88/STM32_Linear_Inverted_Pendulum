# CartPole Swing-Up MuJoCo Simulation

## Mô tả

Mô phỏng điều khiển swing-up cho hệ CartPole inverted pendulum sử dụng MuJoCo.

Chiến lược điều khiển hybrid:
1. **Energy Shaping** (giai đoạn swing-up): Bơm năng lượng để đưa con lắc từ vị trí treo xuống lên vị trí thẳng đứng
2. **LQR Control** (giai đoạn balancing): Giữ cân bằng con lắc ở vị trí upright

## Thông số vật lý (từ Wheeltec IP570)

| Tham số | Giá trị | Đơn vị |
|---------|--------|--------|
| M (khối lượng xe) | 0.26 | kg |
| m (khối lượng con lắc) | 0.04467 | kg |
| l (khoảng cách khớp → CoM) | 0.1 | m |
| J (momen quán tính) | 5.96e-4 | kg·m² |
| g | 9.81 | m/s² |
| dt (bước thời gian) | 0.005 | s |
| F_max | 10.0 | N |

## Tệp quan trọng

- **cartpole.xml** - Mô hình MuJoCo với thông số vật lý thực tế
- **sim_main.py** - Script mô phỏng với điều khiển hybrid
  - `Params` - Cấu hình thông số hệ thống
  - `compute_lqr_gain()` - Tính toán LQR gain từ tuyến tính hóa
  - `energy_shaping()` - Luật điều khiển bơm năng lượng
  - `lqr_control()` - Luật điều khiển LQR
  - `run_simulation()` - Vòng lặp chính với MuJoCo viewer

## Cách chạy

### Chế độ visualize (với MuJoCo viewer)

```bash
cd /home/giap/Projects/STM32-Linear-Inverted-Pendulum
python3 sim/sim_main.py
```

### Điều khiển bằng CLI

```bash
# Chỉnh thời gian mô phỏng
python3 sim/sim_main.py

# Lưu ý: script sử dụng MuJoCo viewer nên cần môi trường đồ họa
```

## Các bước hoạt động

1. **Khởi tạo**: Con lắc ở vị trí treo xuống (θ=0)
2. **Giai đoạn swing-up**: 
   - Kiểm tra |θ - π| > switch_angle (mặc định 0.30 rad ≈ 17°)
   - Áp dụng energy shaping: u = k_E·dE·cos(θ)·θ̇ - k_x·x - k_v·ẋ
3. **Giai đoạn balancing**:
   - Khi |θ - π| < switch_angle, chuyển sang LQR
   - Áp dụng: u = -K·[x, ẋ, θ_norm, θ̇]

## Các công thức chính

### Energy Shaping
```
E = 0.5·J·θ̇² + m·g·l·(cos(θ) + 1)    [tham chiếu tại upright = 0]
u = k_E·(E - E_des)·cos(θ)·θ̇ + u_cart
```

### LQR Linearization (tại θ=π)
Sử dụng momen quán tính J thực tế:
```
denom = (M+m)·J - (m·l)²
A[3,2] = -(M+m)·m·g·l / denom
B[1,0] = J / denom
B[3,0] = m·l / denom
```

## Tham số điều khiển

Trong `Params` class:
- `k_E = 15.0` - Hệ số bơm năng lượng
- `k_x = 2.0` - Phản hồi vị trí xe
- `k_v = 1.5` - Phản hồi vận tốc xe
- `switch_angle = 0.30 rad` - Ngưỡng chuyển sang LQR
- `Q = diag([10, 1, 100, 10])` - Trọng số LQR cho x
- `R = [[1.0]]` - Trọng số LQR cho u

Có thể điều chỉnh các giá trị này trong file để cải thiện hiệu năng.

## Ghi chú

- Convention MuJoCo: θ=0 là treo xuống (hanging), θ=π là thẳng đứng (upright)
- Tất cả góc được chuẩn hóa về [-π, π]
- LQR được tính toán từ tuyến tính hóa chính xác sử dụng J thay vì L²
