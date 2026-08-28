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


```

```
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

```

```

```
### 1. Điều khiển Swing-Up (Pha Phi Tuyến)
* Sử dụng quy luật kích thích hình sin theo tần số dao động riêng của con lắc:
  $$\omega_n = \sqrt{\frac{g}{l}} \approx 6.26 \text{ rad/s}$$
* **Hạn chế biên độ xe:** Bổ sung phản hồi PD ảo để xe lấy đà đối xứng qua lại 2 hướng và giữ hành trình nằm an toàn trong khoảng giới hạn ($x \in [-10, 10]\text{ m}$):
  $$F_{\text{swing}} = F_{\text{osc}} \cdot \sin(\omega_n t) - (k_x x + k_v \dot{x})$$

### 2. Logic Chuyển Mạch (Hysteresis & Latching)
* **Điều kiện bắt góc:** Kích hoạt khi $|\theta| \le 10^\circ$ ($0.1745 \text{ rad}$) và vận tốc góc $|\dot{\theta}| \le 2.5 \text{ rad/s}$.
* **Khử vòng lặp đại số (Algebraic Loop):** Tín hiệu kích hoạt đi qua khối `Memory` trước khi đưa vào chân Reset ($R$) của `PID Angle` và chân điều khiển của khối `Switch`.

### 3. Điều Khiển Cân Bằng Cascade PID (Pha Tuyến Tính)
* **Vòng ngoài (`PID Pos`):** Tính toán góc nghiêng đặt $\theta_{\text{ref}}$ dựa trên sai lệch vị trí $(x_{\text{ref}} - x)$.
* **Vòng trong (`PID Angle`):** Tính toán lực tác động trực tiếp lên xe $u(t)$ để triệt tiêu sai số góc.
* Tích hợp tính năng **External Reset (Rising edge)** và **Saturation / Clamping** để chống hiện tượng bão hòa tích phân (Integrator Windup) khi vừa chuyển chế độ.

---

## 🛠️ Hướng Dẫn Cài Đặt & Chạy Mô Phỏng

### Yêu cầu hệ thống:
* **MATLAB / Simulink** (Khuyến nghị bản R2022a trở lên)
* **Simscape** & **Simscape Multibody**
* **Simulink Control Design**
