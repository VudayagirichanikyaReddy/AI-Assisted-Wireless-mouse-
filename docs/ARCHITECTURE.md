# Architecture

This project is an embedded-system simulation of a wearable IMU-based pointing
device with adaptive sensor-noise reduction.

```text
Wokwi ESP32 + MPU6050 ── IMU,CSV stream ──┐
                                          ├─ Python processing ── Tkinter GUI
Synthetic MPU6050 model ──────────────────┘
```

Both input modes feed the same processing pipeline:

1. input validation and sample timing
2. stationary calibration and bias removal
3. accelerometer/gyroscope complementary fusion
4. motion-dependent adaptive smoothing
5. configurable dead zone and bounded cursor mapping
6. metrics, plots, recording, and benchmark output

The ESP32 firmware only acquires and formats sensor data. Filtering remains in
Python so the algorithm can be inspected and compared during the demonstration.

