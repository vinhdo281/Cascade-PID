# Mô Phỏng Con Lắc Ngược Trên Xe: Swing-Up & Điều Khiển Cân Bằng Bằng Cascade PID (MATLAB/Simulink)

Mô hình mô phỏng hoàn chỉnh hệ thống **Con lắc ngược trên xe (Inverted Pendulum on Cart)** được xây dựng trên nền tảng **MATLAB & Simulink (Simscape Multibody)**. Dự án triển khai chiến lược điều khiển lai (Hybrid Control) kết hợp giữa thuật toán **Swing-Up (Cộng hưởng / Bơm năng lượng)** để kích thích con lắc tự dựng lên từ vị trí treo dưới và cấu trúc **Cascade PID (Position + Angle)** để giữ cân bằng thẳng đứng quanh đỉnh cũng như bám điểm đặt vị trí xe.

---

## 📌 Tổng Quan Hệ Thống

Mô hình đối tượng vật lý gồm một xe truyền động chuyển động tịnh tiến 1 bậc tự do trên ray và một con lắc quay tự do gắn trên xe.

### 📐 Thông Số Vật Lý (Simscape Plant)
* **Khối lượng xe ($M$):** $1.0 \text{ kg}$ (Khối hộp Brick Solid: $0.3 \times 0.15 \times 0.08 \text{ m}$)
* **Khối lượng con lắc ($m$):** $0.15 \text{ kg}$ (Khối trụ Cylindrical Solid: $L = 0.5 \text{ m}, R = 0.015 \text{ m}$)
* **Khoảng cách từ trục quay đến trọng tâm con lắc ($l$):** $0.25 \text{ m}$ ($l = L/2$)
* **Mô-men quán tính con lắc ($J$):** $\frac{1}{3} m L^2 = 0.0125 \text{ kg}\cdot\text{m}^2$
* **Gia tốc trọng trường ($g$):** $9.81 \text{ m/s}^2$

---

## 🕹️ Cấu Trúc & Chiến Lược Điều Khiển

Hệ thống hoạt động qua hai giai đoạn riêng biệt được điều phối bởi **Cơ chế chuyển mạch có trễ (Hysteresis / State Latching)**:
[ Trạng thái ban đầu: Con lắc treo dưới (|theta| > 10°) ]
                                         │
                                         ▼
                              [ Bộ điều khiển Swing-Up ]
                        (Dao động cộng hưởng + Giữ tâm xe)
                                         │
                                         │ Điều kiện bắt góc: |theta| <= 10° & |omega| <= 2.5 rad/s
                                         ▼
                              [ Khối Switch Tự Động ]
                          (Reset tích phân qua khối Memory)
                                         │
                                         ▼
                            [ Bộ Điều Khiển Cascade PID ]
                 ┌───────────────────────────────────────────────┐
                 │  Vòng ngoài: PID Pos   --> Góc đặt theta_ref  │
                 │  Vòng trong: PID Angle --> Lực tác động u(t)  │
                 └───────────────────────────────────────────────┘

                 ### 1. Swing-Up Control (Nonlinear Phase)
* Driven by resonant harmonic excitation at the natural frequency of the pendulum:
  $$\omega_n = \sqrt{\frac{g}{l}} \approx 6.26 \text{ rad/s}$$
* **Centering & Damping Action:** A soft PD feedback term ensures the cart stays strictly within the bounded region ($x \in [-10, 10]\text{ m}$):
  $$F_{\text{swing}} = F_{\text{osc}} \cdot \sin(\omega_n t) - (k_x x + k_v \dot{x})$$

### 2. Switching Logic (Hysteresis & Latching)
* **Catch Condition:** Activates when $|\theta| \le 10^\circ$ ($0.1745 \text{ rad}$) and angular velocity $|\dot{\theta}| \le 2.5 \text{ rad/s}$.
* **Algebraic Loop Prevention:** A `Memory` block delays the reset and switching flag to ensure clean execution and avoid direct feedthrough algebraic loops.

### 3. Cascade PID Balancing (Linear Phase)
* **Outer Loop (Position PID):** Computes the desired lean angle $\theta_{\text{ref}}$ based on position error $(x_{\text{ref}} - x)$.
* **Inner Loop (Angle PID):** Computes the control force $u(t)$ to stabilize the pendulum around $\theta_{\text{ref}} = 0$.
* Configured with **Integrator Reset (Rising Edge)** and **Output Clamping/Saturation** to prevent integrator windup during handoff.

---

## 🛠️ Simulation Setup & Requirements

* **MATLAB / Simulink** (R2022a or newer recommended)
* **Simscape** & **Simscape Multibody**
* **Simulink Control Design**
