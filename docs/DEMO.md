# Five-minute demonstration

1. Run `./run_python.sh` and select **Synthetic / Stationary**.
2. Point out that the raw IMU signal moves slightly while the filtered output
   and cursor remain stable.
3. Press **CALIBRATE** while stationary and show the collected bias values.
4. Select **Tremor** and raise sensor noise. Compare raw jitter, filtered
   jitter, adaptive alpha, and noise reduction.
5. Select **Fast**. Show that smoothing becomes less aggressive and the cursor
   remains responsive.
6. Select **Circular** and show the filtered cursor trajectory.
7. Press **BENCHMARK** and open `logs/benchmark.json` or `logs/benchmark.csv`.
8. If Wokwi CLI is configured, export `WOKWI_CLI_TOKEN`, run
   `./run_integrated.sh`, and show the Wokwi source status.

The fallback demonstration is complete even without Wokwi or physical hardware.

