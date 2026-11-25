# Algorithm

The synthetic source models gravity from roll/pitch and adds Gaussian noise,
bias, slow drift, quantization, gyro noise, finite sampling, and packet loss.

Calibration collects stationary samples. Accelerometer bias is the measured
mean minus `[0, 0, 1]` g; gyro bias is the measured mean angular rate.

For every sample, roll and pitch are estimated from the accelerometer and
integrated from the gyro. A complementary fusion step combines them:

```text
gyro_angle = previous_angle + gyro_rate * dt
fused = alpha_gyro * gyro_angle + (1 - alpha_gyro) * accelerometer_angle
```

The adaptive stage estimates disagreement and recent output change. Small
changes receive stronger exponential smoothing, while large changes receive a
larger smoothing coefficient so intentional fast motion stays responsive. This
is the project's intelligent component: adaptive noise estimation and
motion-dependent filtering, not a neural network.

The dead zone removes small processed angles before cursor mapping. Cursor
coordinates are clamped to 0–100 and are never sent to the operating system.

