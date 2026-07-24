# Real-Time Occupancy Grid SLAM Engine on QNX RTOS

A lightweight, bare-metal 2D Occupancy Grid SLAM engine built from scratch in C on the **QNX Real-Time Operating System**. The system ingests 1,024-ray LiDAR point clouds over TCP, synchronizes telemetry via zero-copy shared memory, and generates a persistent spatial map using integer-based Bresenham ray-casting.

> **Achievement:** Selected among the **Top 10 Projects** in the QNX RTOS course cohort.

---

## 🏗️ System Architecture

The project is structured around a decoupled **Producer-Consumer architecture** using Inter-Process Communication (IPC) to enforce real-time determinism:

1. **Simulation Layer (PyBullet / Python):** Simulates UAV dynamics undergoing orbital ("planetary") motion in PyBullet. It broadcasts 1,024-ray LiDAR scans as 12,296-byte binary payloads over **UDP** at 10 Hz to the QNX target machine running inside a VMware virtual machine environment.
2. **Telemetry Receiver (QNX Target Process 1):** Binds to Port 5000, uses `recv()` with `MSG_WAITALL` for zero-copy atomic packet reception, and writes data to Shared Memory (`/dev/shm`).
3. **SLAM Engine (QNX Target Process 2):** Consumes SHM data under POSIX Semaphore locks, applies world-to-grid coordinate transformations, and updates an 8-bit probabilistic occupancy grid.

---

## 🔑 Key Engineering Highlights

* **Deterministic Synchronization:** Uses POSIX semaphores to eliminate race conditions between high-frequency socket reads and grid updates without blocking the RTOS kernel.
* **Kinematic Offset Compensation:** Converts continuous orbital world coordinates into a fixed $200 \times 200$ matrix, anchoring $(0,0)$ at the center array offset to safely handle negative coordinate trajectories without boundary overflows.
* **Resource-Optimized Geometry:** Implements an integer-only **Bresenham’s Line Algorithm** for ray tracing, avoiding expensive floating-point arithmetic on the target board.
* **Probabilistic Noise Filtering:** Uses an `int8_t` confidence scoring system (hit-increment / miss-decrement) to filter out transient simulation noise and build persistent spatial obstacles.
* **Low-Overhead Terminal Rendering:** Live ANSI escape cursor resetting (`\033[H`) paired with a frame-rate throttle to display live-updating maps directly in the QNX console.

---

## 🛠️ Technical Specifications

| Parameter | Specification |
| :--- | :--- |
| **Operating System** | QNX RTOS |
| **Grid Resolution** | $200 \times 200$ cells ($10\text{m} \times 10\text{m}$ area at $0.05\text{m}$ resolution) |
| **Memory Footprint** | $40,000$ bytes (`int8_t` grid array) |
| **IPC Mechanism** | POSIX Shared Memory (`shm_open`) + Semaphores (`sem_open`) |
| **Language Stack** | C (QNX C99 Standard), Python 3 (PyBullet Simulation) |

---

## 🚀 How to Run

1. **Start the QNX Receiver & SLAM Engine:**
   Import `telemetry_receiver` and `slam_engine` into QNX Momentics IDE, build for your target architecture, and launch both binaries on the QNX system.
   
2. **Start the Telemetry Simulator:**
   ```bash
   cd sim
   pip install -r requirements.txt
   python drone_sim.py --ip <QNX_TARGET_IP> --port 5000
