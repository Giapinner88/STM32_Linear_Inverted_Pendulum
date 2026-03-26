# Phase 1 Hardware Measurements

## 1. Motor Dead-Zone Characterization (26/03/2026)

### Forward Direction Test
- **Procedure**: OLED shows PWM values starting from 0, incrementing by 150 per 800ms
- **Observation**: Watch motor shaft/wheel for first visible rotation
- **Record**: PWM value when motion **first appears**

| Trial | Start PWM | Percentage |
|-------|-----------|-----------|
| 1     |           | %         |
| 2     |           | %         |
| 3     |           | %         |
| **Average** | **---** | **---** |

**Forward Dead-Zone (PWM units)**: `---`
**Forward Dead-Zone (%)**: `--- / 7199 × 100`

---

### Reverse Direction Test
- **Procedure**: Same as forward, but motor direction reversed via GPIO
- **Observation**: Watch for first rotation in opposite direction
- **Record**: PWM value when motion **first appears**

| Trial | Start PWM | Percentage |
|-------|-----------|-----------|
| 1     |           | %         |
| 2     |           | %         |
| 3     |           | %         |
| **Average** | **---** | **---** |

**Reverse Dead-Zone (PWM units)**: `---`
**Reverse Dead-Zone (%)**: `--- / 7199 × 100`

---

### Analysis
```
For LQR Compensation:
If forward dead zone = d_f, reverse dead zone = d_r

Average dead zone: d_avg = (d_f + d_r) / 2
Dead zone symmetric?: |d_f - d_r| < 100 PWM units?

Compensation formula (in Phase 2):
if |u_ref| < d_avg:
    u_motor = 0
else:
    u_motor = u_ref + sign(u_ref) × d_avg
```

---

## 2. Encoder Calibration (To-Do)

**Objective**: Measure linear distance per encoder pulse count

| Trial | Distance (m) | Pulse Count | k_x factor |
|-------|--------------|------------|-----------|
| 1     | 1.00         |            | m/pulse   |
| 2     | 1.00         |            | m/pulse   |
| 3     | 1.00         |            | m/pulse   |
| **Average** | **1.00** | **---** | **---** |

**Measured k_x**: `--- m/pulse`
**Status**: ❌ Not yet tested

---

## 3. MPU6050 Calibration (To-Do)

### Sensor Offsets (placed on level surface, stationary)

**Accelerometer** (100 samples average):
```
ax_offset = --- m/s²
ay_offset = --- m/s²
az_offset = --- m/s² (should be ~9.81)
```

**Gyroscope** (100 samples average):
```
gx_offset = --- deg/s
gy_offset = --- deg/s
gz_offset = --- deg/s
```

### Reference Frame Verification
- [ ] Positive theta = pendulum tilts **forward** (toward front of cart)
- [ ] Positive x_dot = cart moves **forward**
- [ ] Angle zero = pendulum vertical downward
- [ ] Verified via: ______

**Status**: ❌ Not yet tested

---

## 4. System Verification Checklist

- [x] LED PA4 blinks 500ms on/off
- [x] Motor direction reversible via GPIO (BIN1/BIN2)
- [x] Motor PWM control 0-100% via TIM3 compare
- [x] OLED displays real-time values
- [x] Motor forward PWM dead-zone measured
- [x] Motor reverse PWM dead-zone measured
- [ ] Encoder calibrated (distance vs pulses)
- [ ] MPU6050 offsets measured
- [ ] Reference frame verified (consistent signs)

---

## Notes
- Test must be running continuously; OLED shows PWM every 800ms
- Record 3 trials for averaging, discard outliers >100 PWM units
- If forward dead-zone is 50-100 PWM (0.7-1.4%), that's typical for hobby motors
- If asymmetric >200 PWM units, may need motor inspection or controller tuning
