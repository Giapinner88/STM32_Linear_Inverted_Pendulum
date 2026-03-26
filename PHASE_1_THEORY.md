# Giai Đoạn 1: Hardware & Xác Minh - Lý Thuyết & Triển Khai

**Ngày bắt đầu:** 26/03/2026  
**Mục tiêu:** Xác minh & lượng hóa các đặc tính phần cứng thực tế

---

## I. Nền Tảng Lý Thuyết

### I.1 Dead-Zone Compensation (Bù Vùng Chết)

#### 📌 Vấn Đề
Động cơ DC có chổi than có **vùng chết (dead-zone)**: tồn tại một ngưỡng điện áp tối thiểu $U_{th}$ mà dưới đó động cơ không chạy. 

**Nguyên nhân vật lý:**
- Lực hấp dẫn tĩnh (Static friction): $\tau_{static} = \mu_s \cdot N$
- Lực ly tâm trong cuộn dây
- Điện trở hạt chổi

#### 📊 Mô Hình Dead-Zone

Với tín hiệu điều khiển lý tưởng $u \in [-1, 1]$ (chuẩn hóa):

$$\tau_{real}(u) = \begin{cases}
-\tau_{max} & \text{if } u < -u_{th} \\
0 & \text{if } |u| \leq u_{th} \\
u_{max} & \text{if } u > u_{th}
\end{cases}$$

Trong đó:
- $u_{th}$ = ngưỡng dead-zone (thường 0.15 - 0.30)
- $\tau_{max}$ = mô-men tối đa của động cơ

#### ✅ Giải Pháp: Linear Compensation

Ánh xạ tuyến tính từ vùng chết:

$$u_{pwm} = \begin{cases}
-1.0 & \text{if } u \leq -u_{th} \\
\frac{u - u_{th}}{1 - u_{th}} & \text{if } -u_{th} < u < u_{th} \\
1.0 & \text{if } u \geq u_{th}
\end{cases}$$

**Lợi ích:**
- Loại bỏ hiệu ứng stick-slip
- Tránh chattering quanh điểm cân bằng trong LQR
- Đáp ứng nhanh hơn

---

### I.2 Hệ Quy Chiếu (Reference Frame Alignment)

#### 📌 Vấn đề  
Ba cảm biến (Motor, Encoder, MPU6050) có thể **không cùng hướng** → dữ liệu nhầm lẫn.

#### 🔄 Đồng Bộ Hệ Quy Chiếu

**Điều kiện phải thoả mãn:**

| Cảm Biến | Tín Hiệu | Ý Nghĩa Dương | Kiểm Thử |
|----------|----------|---------------|---------|
| **Encoder (x)** | $\dot{x}$ | Xe tới **phía trước** | Lực bánh xe về phía trước → $\dot{x} > 0$ |
| **MPU6050 (θ)** | $\theta$ | Con lắc **nghiêng về trước** | Kéo gạt con lắc về trước → $\theta > 0$ |
| **Motor (u)** | $u > 0$ | Bánh xe **quay về phía trước** | Lệnh $u > 0$ → Motor quay dạo, xe tiến |

#### 🔧 Cách Hiệu Chỉnh
Nếu phát hiện tín hiệu **ngược chiều:**
```c
// Nếu encoder bị ngược:
velocity = -raw_velocity;

// Nếu MPU6050 bị ngược:
theta = -raw_theta;

// Nếu motor bị ngược:
u = -u_commanded;
```

---

### I.3 Phương Trình Động Lực Học (Dynamics)

#### 📐 Mô Hình Tuyến Tính (Linear Model)

Đối với **xe song bánh + con lắc đơn**:

$$\begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & -\alpha g & -\beta & -\gamma \\ 0 & \delta & \epsilon & -\zeta \end{bmatrix} \begin{bmatrix} x \\ \theta \\ \dot{x} \\ \dot{\theta} \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ K_x \\ K_\theta \end{bmatrix} u$$

Với:
- $x$ = vị trí tuyệt đối của xe (m)
- $\theta$ = góc con lắc từ phương thẳng đứng (rad)
- $u$ = lực đẩy từ động cơ (N)
- $\alpha, \beta, \gamma, \delta, \epsilon, \zeta$ = hệ số từ khối lượng, moment quán tính, ...

**Nhận xét:** Hệ thống **không ổn định tự nhiên** ($\theta = \pi$ là điểm cân bằng không ổn định).

---

### I.4 Diode Karush-Kuhn-Tucker (KKT) cho Dead-Zone

Khi LQR thiết kế, **phải xem xét dead-zone** trong hàm mất mát:

$$J = \int_0^\infty (x^T Q x + u^T R u) dt$$

Với ràng buộc:

$$|u| \geq u_{th} \quad \text{hoặc} \quad u = 0$$

**Giải pháp thực hành:** Dùng hàm saturation + dead-zone:

$$u_{compensated}(u) = \begin{cases}
-sat(\frac{|u| - u_{th}}{1 - u_{th}}) & \text{if } u < -u_{th} \\
0 & \text{if } |u| \leq u_{th} \\
sat(\frac{u - u_{th}}{1 - u_{th}}) & \text{if } u > u_{th}
\end{cases}$$

---

## II. Kế Hoạch Thực Thi (Implementation Plan)

### Phase 1.1: Đo Dead-Zone Motor

**Số liệu cần đo:**
- Ngưỡng PWM tối thiểu ($PWM_{min}$) để bánh xe bắt đầu chuyển động
- Tốc độ angle của PWM ở giai đoạn tuyến tính
- Độ trễ (delay) từ lệnh tới phản hồi

**Phương pháp:**
1. Ghi lại PWM từ 0% → 100% từng bước 5%
2. Để xe trên giá, không chạm đất
3. Quan sát motor bắt đầu quay ở PWM nào
4. Ghi nhận thời điểm + vận tốc góc

**Công thức:**
$$u_{th} = \frac{PWM_{min}}{PWM_{max}}$$

---

### Phase 1.2: Kiểm Chứng Encoder

**Số liệu cần đo:**
- Hệ số chuyển đổi encoder ticks → vị trí (m)
- Độ phân giải (resolution) của encoder
- Kiểm tra hướng dương (CW = tới phía trước?)

**Phương pháp:**
1. Kéo xe đi quãng đường $d$ (ví dụ: 1 mét)
2. Đếm số xung encoder
3. Tính hệ số: $k_x = \frac{d}{\text{pulse count}}$

**Công thức:**
$$\dot{x} = k_x \cdot \omega_{encoder}$$

---

### Phase 1.3: Hiệu Chỉnh MPU6050 (Gyroscope + Accelerometer)

**Số liệu cần đo:**
- Offset (bias) của gyroscope
- Offset của accelerometer
- Góc pitch khi xe trên mặt phẳng ngang

**Phương pháp:**
1. Để xe yên (không chuyển động)
2. Ghi 100 mẫu gyro + accel
3. Tính trung bình → đó là offset
4. Trừ offset khỏi dữ liệu thô

**Bộ lọc Complimentary:**
$$\theta = 0.98 \cdot (\theta_{prev} + \omega_{gyro} \cdot dt) + 0.02 \cdot \theta_{accel}$$

---

### Phase 1.4: Kiểm Thử Đồng Bộ Hệ Quy Chiếu

**Bảng Kiểm Thử:**

| Test | Hành Động | Kỳ Vọng | Thực Tế | ✅/❌ |
|------|-----------|---------|---------|------|
| **Encoder** | Kéo xe tới phía trước 0.5m | $x > 0$ | ? | ? |
| **Gyro** | Kéo gạt con lắc về phía trước 5° | $\theta > 0$ | ? | ? |
| **Motor** | Gửi $u = +50\%$ | Motor quay dạo, xe tiến | ? | ? |
| **Motor** | Gửi $u = -50\%$ | Motor quay ngược, xe lùi | ? | ? |

---

## III. Công Thức & Hằng Số

### Hằng Số Hệ Thống Cần Đo

| Hằng Số | Ký Hiệu | Đơn Vị | Ước Tính | Ghi Chú |
|---------|---------|---------|----------|---------|
| Khối lượng xe | $M$ | kg | 1.5 | |
| Khối lượng con lắc | $m$ | kg | 0.3 | |
| Chiều dài con lắc | $l$ | m | 0.4 | Từ khớp đến trọng tâm |
| Dead-zone | $u_{th}$ | % | 15-20% | Sẽ đo |
| Hệ số encoder | $k_x$ | m/pulse | ? | Sẽ đo |
| Bán kính bánh xe | $r$ | m | 0.05 | |

---

## IV. Timeline Giai Đoạn 1

| Ngày | Nhiệm Vụ | Trạng Thái |
|------|----------|-----------|
| 26/03 | Viết code đo dead-zone motor | 🔄 |
| 26/03 | Hiệu chỉnh encoder | 🔄 |
| 27/03 | Hiệu chỉnh MPU6050 | ⏳ |
| 27/03 | Kiểm chứng đồng bộ hệ quy chiếu | ⏳ |
| 28/03 | **Giai Đoạn 1 hoàn tất** ✅ | ⏳ |

---

## V. Tài Liệu Tham Khảo

- [STM32F103C8 Datasheet](https://www.st.com/en/microcontrollers/)
- MG513 DC Motor Spec Sheet
- MPU6050 I2C Protocol (InvenSense)
- Complementary Filter Theory (Mahony et al., 2008)

---

**Cập nhật lần cuối:** 26/03/2026  
**Trạng thái:** 🟡 Đang triển khai Giai Đoạn 1
