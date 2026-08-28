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
