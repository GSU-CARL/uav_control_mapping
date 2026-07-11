## on ground

```bash
────────────────────────────────────────────────────────────────────────────────
  ── SYNC SUMMARY (last 10 scans, scan #10) ──
────────────────────────────────────────────────────────────────────────────────
  IMU actual rate  : 251.7 Hz (expected 250 Hz)
  LiDAR actual rate: 10.0 Hz (expected 10 Hz)

  Δ(LiDAR hdr − nearest IMU) [ms]:  mean=  +0.00  std=  0.00  min=  +0.00  max=  +0.00
  LiDAR wall-arrival gap [ms]:       mean=+100.03  std=  6.09  min= +86.77  max=+109.11
  Per-point time span [ms]:          mean=  +0.00  std=  0.00  min=  +0.00  max=  +0.00
  Expected pt.time span              100.0 ms  → MISMATCH — check rpm/h_samples

  Diagnosis hints:
  ✓ IMU-LiDAR timestamp delta looks OK (<15 ms)
  ✗ Per-point time span (0.0 ms) ≠ expected (100.0 ms).
    → Check rpm (6 s/rev) and horizontal_samples in lidar_time_injector.
    → Also check timestamp_unit: 0 (seconds) in velody16.yaml.

```

## take off

```bash
────────────────────────────────────────────────────────────────────────────────
  ── SYNC SUMMARY (last 10 scans, scan #310) ──
────────────────────────────────────────────────────────────────────────────────
  IMU actual rate  : 247.5 Hz (expected 250 Hz)
  LiDAR actual rate: 9.9 Hz (expected 10 Hz)

  Δ(LiDAR hdr − nearest IMU) [ms]:  mean=  +0.00  std=  0.00  min=  +0.00  max=  +0.00
  LiDAR wall-arrival gap [ms]:       mean=+100.95  std=  7.58  min= +47.43  max=+150.35
  Per-point time span [ms]:          mean=  +0.00  std=  0.00  min=  +0.00  max=  +0.00
  Expected pt.time span              100.0 ms  → MISMATCH — check rpm/h_samples

  Diagnosis hints:
  ✓ IMU-LiDAR timestamp delta looks OK (<15 ms)
  ✗ Per-point time span (0.0 ms) ≠ expected (100.0 ms).
    → Check rpm (6 s/rev) and horizontal_samples in lidar_time_injector.
    → Also check timestamp_unit: 0 (seconds) in velody16.yaml.
```