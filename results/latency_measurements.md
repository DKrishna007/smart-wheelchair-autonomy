# Latency Measurements

## End-to-End Pipeline Latency (Jetson AGX Orin)

| Test Run | Min (ms) | Max (ms) | Mean (ms) | Std (ms) |
|----------|----------|----------|-----------|----------|
| Run 1 | 78 | 119 | 94 | 8.2 |
| Run 2 | 82 | 115 | 91 | 7.8 |
| Run 3 | 80 | 122 | 96 | 9.1 |
| Run 4 | 77 | 118 | 93 | 8.5 |
| **Average** | **79** | **119** | **93.5** | **8.4** |

Target: <150ms for safe human-in-the-loop wheelchair control.
Result: All runs within 80-120ms target. PASS

## Collision Frequency Over Time

| Session | Duration | Collisions | Rate (per hr) |
|---------|----------|------------|---------------|
| Baseline (no autonomy) | 2 hr | 16 | 8.0 |
| DQN only | 2 hr | 8 | 4.0 |
| DQN + A* + safety stop | 2 hr | 5 | 2.5 |
| **Final system** | **4 hr** | **9** | **2.25** |

Improvement vs baseline: **72% reduction** PASS
