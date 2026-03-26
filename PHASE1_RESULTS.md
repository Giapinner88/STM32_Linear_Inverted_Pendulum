# Phase 1: Motor Travel Range Characterization

## Test Results

**Date**: March 26, 2026  
**Platform**: STM32F103C8T6 @ 72MHz  
**Motor**: DC Motor with Quadrature Encoder (TIM4 encoder mode)  
**PWM Duty**: 50% (3500/7199)

### Measurements

| Parameter | Value | Unit |
|-----------|-------|------|
| **LEFT WALL Position** | 4162 | encoder pulses |
| **RIGHT WALL Position** | 65502 | encoder pulses |
| **Total Travel Range** | 61340 | encoder pulses |

### Test Procedure

1. **LEFT Phase**: 
   - Motor runs at 50% PWM toward left wall
   - Collision detection: encoder stable for 3 readings (~900ms) = stops
   - Captures: 4162 pulses

2. **RIGHT Phase**:
   - Motor continues from left position, reverses direction
   - Runs toward right wall at 50% PWM
   - Collision detection: encoder stable for 3 readings = stops
   - Captures: 65502 pulses

### Key Findings

- Motor successfully spans **61340 encoder pulses** from left to right wall
- Encoder counter is 16-bit (0-65535), near max at right wall (65502)
- Collision detection via stability counting works reliably
- No issues with motor reversing or direction control

### Next Steps

1. **Calibration**: Measure wall-to-wall distance (meters) to get conversion factor
   - **Conversion**: [distance in meters] / 61340 pulses = meters/pulse
   
2. **Dead-zone Testing**: Determine minimum PWM needed for motor to start moving
   
3. **LQR Controller Design**: Use travel range for state boundaries and control limits

---

## Hardware Configuration

- **Motor Control**: TIM3 Channel 4 (PWM), GPIO PB13/PB12 (direction)
- **Encoder Input**: TIM4 in encoder mode, GPIO PA0/PA1
- **Communication**: USART1 @ 128000 baud
- **Display**: SSD1306 OLED 128x64
