# AI-Assisted Wireless Air Mouse Glove

An embedded-system simulation of a wearable IMU-based pointing device with
adaptive sensor-noise reduction. The project intentionally uses explainable
signal processing rather than a neural network or machine-learning model.

## Layers
- `wokwi/`: ESP32 + MPU6050 embedded simulation and serial protocol.
- `python/air_mouse.py`: adaptive sensor-fusion, denoising, cursor mapping, metrics and GUI.
- `logs/`: CSV recordings and JSON benchmark reports.

## Run and test
```bash
cd ~/air_mouse_glove
./start.sh
./run_python.sh
./test.sh
./benchmark.sh
```

`test.sh` is headless and prints `SELF TEST PASSED`. The benchmark uses the
same engine as the GUI and writes `logs/benchmark.json` and
`logs/benchmark.csv`.

## Simulation workflow
1. Run the Python GUI using `Synthetic` input.
2. Select Tremor and increase Sensor Noise.
3. Compare Raw vs Filtered orientation and the cursor paths.
4. Run Calibration while stationary.
5. Use Benchmark to create `logs/benchmark_*.json`.
6. Use Record to create a CSV dataset.
7. For Wokwi CLI/RFC2217, export `WOKWI_CLI_TOKEN`, run `./run_integrated.sh`,
   and use the Wokwi source in the GUI. Without the token or CLI, Synthetic
   mode remains fully functional.

## Wokwi

Build the embedded project with:

```bash
export WOKWI_CLI_TOKEN=your_token
cd wokwi && pio run
```

For the full CLI workflow, from the project root run:

```bash
export WOKWI_CLI_TOKEN=your_token
./run_integrated.sh
```

The RFC2217 endpoint is `rfc2217://localhost:4000`. If it is unavailable, the
dashboard stays usable in Synthetic mode rather than crashing.

PlatformIO and Matplotlib caches are stored in `.platformio/` and `.cache/`
inside the project so the workflow works without writable global tool config.

Additional design notes are in `docs/ARCHITECTURE.md`,
`docs/ALGORITHM.md`, and `docs/DEMO.md`.

## Algorithm
The adaptive layer is deliberately lightweight: accelerometer/gyro complementary fusion, motion/noise estimation, adaptive exponential smoothing and a dead zone. It is an adaptive signal-processing layer, not a claimed neural network.
