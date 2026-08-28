# Mô Phỏng Con Lắc Ngược Trên Xe: Điều Khiển Cân Bằng Bằng Cascade PID (MATLAB/Simulink)

Mô hình mô phỏng hệ thống **Con lắc ngược trên xe (Inverted Pendulum on Cart)** được xây dựng trên nền tảng **MATLAB & Simulink (Simscape Multibody)**. Dự án tập trung vào cấu trúc điều khiển phân tầng **Cascade PID (Position + Angle)** để duy trì con lắc ở vị trí cân bằng thẳng đứng quanh đỉnh ($\theta = 0$) và bám chính xác điểm đặt vị trí của xe ($x$).

---

## 📌 Tổng Quan Hệ Thống

Mô hình đối tượng vật lý gồm một xe truyền động chuyển động tịnh tiến 1 bậc tự do dọc theo ray trượt và một thanh con lắc quay tự do gắn trên xe.

### 📐 Thông Số Vật Lý (Simscape Plant)
* **Khối lượng xe ($M$):** $1.0 \text{ kg}$ (Khối hộp Brick Solid: $0.3 \times 0.15 \times 0.08 \text{ m}$)
* **Khối lượng con lắc ($m$):** $0.15 \text{ kg}$ (Khối trụ Cylindrical Solid: $L = 0.5 \text{ m}, R = 0.015 \text{ m}$)
* **Khoảng cách từ trục quay đến trọng tâm con lắc ($l$):** $0.25 \text{ m}$ ($l = L/2$)
* **Mô-men quán tính con lắc ($J$):** $\frac{1}{3} m L^2 = 0.0125 \text{ kg}\cdot\text{m}^2$
* **Gia tốc trọng trường ($g$):** $9.81 \text{ m/s}^2$

---

## 🕹️ Cấu Trúc Điều Khiển Cascade PID

Hệ thống sử dụng vòng lặp kép (Cascade Control Loop) để xử lý bản chất phi tuyến và phi cực tiểu của hệ con lắc ngược:
```
x_ref           e_x     ┌───────────┐  theta_ref     e_theta ┌───────────┐   u(t)   ┌──────────────┐          x, theta
────(+)──────────(─)────►│  PID Pos  ├─────(+)────────(─)────►│ PID Angle ├─────────►│ Simscape Xe  ├───┬────────────────►
     ▲                   └───────────┘      ▲                 └───────────┘ (Lực F)  │   & Con Lắc  │   │
     │                                      │                                        └──────┬───────┘   │
     │                                      └────────────────── q (Angle) ──────────────────┼───────────┘
     └───────────────────────────────────────────────────────── x (Pos) ────────────────────┘
```
### 1. Vòng Ngoài - Điều Khiển Vị Trí Xe (`PID Pos`)
* **Nhiệm vụ:** Tạo ra góc nghiêng đặt $\theta_{\text{ref}}$ cho con lắc dựa trên sai lệch vị trí $e_x = x_{\text{ref}} - x$.
* **Đặc tính:** 
  * Hoạt động ở tần số thấp hơn vòng trong.
  * Tích hợp khâu bão hòa góc (**Saturation Limit**: $\pm 0.1 \div 0.15\text{ rad}$) kèm thuật toán chống bão hòa tích phân (**Anti-windup: Clamping**) nhằm đảm bảo con lắc không bị nghiêng quá giới hạn tuyến tính.

### 2. Vòng Trong - Điều Khiển Cân Bằng Góc (`PID Angle`)
* **Nhiệm vụ:** Tính toán trực tiếp lực tác động $u(t)$ truyền vào xe để triệt tiêu sai lệch góc $e_\theta = \theta_{\text{ref}} - \theta$.
* **Đặc tính:** 
  * Vòng phản hồi tốc độ cao, đóng vai trò tạo độ cứng vững và ổn định hệ thống.
  * Tích hợp bộ lọc vi phân số hạng đạo hàm ($N$) để khử nhiễu đo lường và làm mượt lực điều khiển.

---

## 🛠️ Hướng Dẫn Cài Đặt & Chạy Mô Phỏng

### Yêu cầu hệ thống:
* **MATLAB / Simulink** (Khuyến nghị bản R2022a trở lên)
* **Simscape** & **Simscape Multibody**
* **Simulink Control Design**
