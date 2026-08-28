---
title: "Điều Khiển Robot Tự Cân Bằng · Xây Dựng Bộ Điều Khiển Cascade PID (Vòng Vị Trí – Vòng Góc Nghiêng) Trên Simulink"
excerpt: "Hướng dẫn chi tiết cách xây dựng subsystem Cascade PID hai vòng (Position → Angle) cho robot hai bánh tự cân bằng trong Simulink, từ sơ đồ khối, thông số bộ điều khiển cho tới kiểm chứng đáp ứng."
date: 2026-08-28
tags: [cascade-pid, self-balancing-robot, control-systems, simulink, matlab, esp32, simplefoc]
categories: [blog, control-systems]
header:
  teaser: /images/cascade-pid-robot/diagram_full_control.png
toc: true
toc_label: "Mục Lục"
---
 
Đối với các hệ **con lắc ngược di động (mobile inverted pendulum)** như robot hai bánh tự cân bằng, chỉ dùng một vòng PID duy nhất trên góc nghiêng $\theta$ thường không đủ: robot sẽ đứng thẳng được, nhưng lại **trôi tự do** theo một hướng bất kỳ vì không có phản hồi nào ràng buộc vị trí của nó.
 
Lời giải kinh điển là **kiến trúc Cascade PID hai vòng lồng nhau (nested-loop)**:
 
- **Vòng ngoài (Outer Loop) — `PID Pos`:** phản hồi vị trí/vận tốc bánh xe, xuất ra một **góc nghiêng tham chiếu nhỏ** $\theta_{ref}$.
- **Vòng trong (Inner Loop) — `PID Angle`:** phản hồi góc nghiêng thực tế đo từ IMU, bám theo $\theta_{ref}$ do vòng ngoài cấp, xuất ra tín hiệu điều khiển mô-men/duty cho động cơ.
Bài viết này trình bày lại toàn bộ subsystem `Cascade` đã xây dựng trong Simulink cho bài toán trên: cấu trúc khối, thông số hai bộ PID, cơ chế chuyển chế độ thử nghiệm bằng `Manual Switch`, và cách đọc kết quả mô phỏng.
 
---
 
## 1. Kiến Trúc Điều Khiển Cascade
 
Nguyên tắc cốt lõi: **vòng trong phải nhanh hơn vòng ngoài rất nhiều lần** — vòng góc nghiêng phải ổn định gần như tức thời để vòng vị trí (chậm hơn) có thể "nương" vào nó mà điều khiển chuyển động tổng thể.
 
```
Setpoint vị trí ──▶ (Sum) ──▶ [ PID Pos ] ──▶ θ_ref ──▶ (Sum) ──▶ [ PID Angle ] ──▶ Lực/Duty ──▶ Plant (robot)
      ▲                                              ▲                                        │
      │                                              │                                        │
      └───────────────── phản hồi vị trí x ◀──────────┴────────────── phản hồi góc θ ◀─────────┘
```
 
Sai số của từng vòng:
 
$$e_{pos}(t) = x_{desired}(t) - x(t) \qquad e_{angle}(t) = \theta_{ref}(t) - \theta(t)$$
 
Đầu ra vòng trong (lực/duty điều khiển động cơ):
 
$$u(t) = K_p\, e_{angle}(t) + K_i \int_0^t e_{angle}(\tau)\,d\tau + K_d \frac{N}{1+N\frac{1}{s}}\frac{d\,e_{angle}(t)}{dt}$$
 
---
 
## 2. Subsystem `Cascade` Trong Simulink
 
Toàn bộ vòng điều khiển được đóng gói trong một Subsystem tên `Cascade`, kết nối với mô hình vật lý (plant) thông qua các khối chuyển đổi tín hiệu vật lý `Simulink-PS Converter` / `PS-Simulink Converter` — tức phần điều khiển này giao tiếp với một **mạng vật lý (Physical Network / Simscape)** dựng plant của robot.
 
**Danh sách khối bên trong Subsystem `Cascade`:**
 
| Nhóm chức năng | Khối sử dụng | Số lượng |
|---|---|---|
| Bộ điều khiển | `PID 1dof` (PID Angle, PID Pos) | 2 |
| Tạo tín hiệu tham chiếu | `Step`, `Constant` | 4 + 4 |
| Tính sai số | `Sum` | 4 |
| Chuyển chế độ thử nghiệm | `Manual Switch`, `Switch` | 5 + 1 |
| Giao tiếp Physical Network | `Simulink-PS Converter`, `PS-Simulink Converter` | 2 + 4 |
| Xử lý tín hiệu / logic | `MATLAB Function`, `Memory`, `Clock` | 1 + 1 + 1 |
| Quan sát | `Scope`, `Main Scope` | 2 |
 
### Bước 2.1: Vòng Trong — `PID Angle`
 
- Nhận sai số góc nghiêng $e_{angle} = \theta_{ref} - \theta$ (đầu vào `Angle` được cấp trực tiếp qua `Constant`/`Step` khi thử nghiệm độc lập).
- Xuất tín hiệu điều khiển sang `Sum` để cộng/trừ với các thành phần khác, rồi đi qua `Simulink-PS Converter` để cấp vào mạng vật lý (mô-men động cơ tác động lên thân robot).
- Có `Manual Switch` cho phép **ngắt vòng trong ra khỏi vòng ngoài** để tinh chỉnh riêng $K_p, K_i, K_d$ của khâu góc trước, trước khi ghép vòng ngoài vào — đúng quy trình tuning cascade chuẩn (tune vòng trong trước, vòng ngoài sau).
### Bước 2.2: Vòng Ngoài — `PID Pos`
 
- Nhận sai số vị trí/vận tốc từ cảm biến qua `PS-Simulink Converter`, tính bởi khối `Sum`.
- Đầu ra được **giới hạn biên độ (saturation)** để không sinh ra góc tham chiếu quá lớn khiến robot mất ổn định — đây là điểm khác biệt quan trọng so với vòng trong.
- Kết quả cộng vào setpoint góc của vòng trong thông qua khối `Sum` kế tiếp, tạo thành $\theta_{ref}$.
### Bước 2.3: Khối `Manual Switch` — Chọn Chế Độ Thử Nghiệm
 
5 khối `Manual Switch` trong subsystem cho phép chuyển đổi nhanh giữa các kịch bản kiểm thử mà không cần sửa lại sơ đồ:
 
- **Không lực / Open-loop:** ngắt hoàn toàn PID để kiểm tra hành vi tự nhiên (mất ổn định) của plant.
- **Chỉ vòng trong (Angle-only):** khoá vòng ngoài, chỉ chạy PID Angle để cân bằng tại chỗ.
- **Cascade đầy đủ:** kích hoạt cả hai vòng, robot vừa giữ thăng bằng vừa bám vị trí/vận tốc mong muốn.
Khối `Switch` (dùng tiêu chí `u2 > Threshold`) đóng vai trò logic chuyển mạch dựa trên tín hiệu điều kiện, phối hợp với `MATLAB Function` và `Memory` để xử lý các trường hợp biên (ví dụ giữ trạng thái trước đó khi chuyển chế độ).
 
### Bước 2.4: Kích Thích Đầu Vào Bằng `Step`
 
4 khối `Step` tạo các tín hiệu bậc thang làm setpoint hoặc nhiễu thử nghiệm, thời điểm kích hoạt và biên độ được đặt riêng cho từng kịch bản (đổi hướng di chuyển đột ngột, mô phỏng việc đẩy nhẹ vào robot, v.v.).
 
---
 
## 3. Thông Số Hai Bộ Điều Khiển PID
 
Cả `PID Angle` và `PID Pos` đều dùng khối `PID 1dof` ở dạng **Parallel, liên tục theo thời gian (Continuous-time)**, tự động tính từ **Transfer Function Based (PID Tuner App)**.
 
| Thông số | `PID Angle` (vòng trong) | `PID Pos` (vòng ngoài) |
|---|---|---|
| Dạng bộ điều khiển | PID – Parallel | PID – Parallel |
| Miền thời gian | Continuous-time | Continuous-time |
| $K_p$ | −337.285 | −0.01464 |
| $K_i$ | −1172.513 | −0.001636 |
| $K_d$ | −22.474 | −0.02909 |
| Hệ số lọc $N$ | 110.24 | 189.76 |
| Giới hạn đầu ra | Không giới hạn | **±0.2** (Limit Output = on) |
| Anti-windup | none | none |
 
> **Nhận xét:** vòng trong (`PID Angle`) có độ lợi lớn hơn nhiều bậc so với vòng ngoài (`PID Pos`) — đúng bản chất cascade: vòng trong phải phản ứng "mạnh và nhanh" với sai lệch góc rất nhỏ (đơn vị rad), trong khi vòng ngoài chỉ cần sinh ra một góc tham chiếu nhỏ, bị giới hạn cứng ở ±0.2 (rad) để không bao giờ yêu cầu vòng trong nghiêng quá mức làm robot đổ.
 
Dấu **âm** của toàn bộ các hệ số $K_p, K_i, K_d$ phản ánh quy ước chiều dương của $\theta$ và chiều dương lực/duty được định nghĩa ngược nhau trong mô hình — cần lưu ý khi đối chiếu với phương trình điều khiển ở trên hoặc khi hiện thực lại trên firmware (ví dụ ESP32 + SimpleFOC), nơi có thể quy ước dấu khác.
 
---
 
## 4. Vì Sao Phải Tune Theo Thứ Tự Trong → Ngoài?
 
Nếu tune đồng thời cả hai vòng, hệ số của vòng ngoài sẽ liên tục "đuổi theo" một plant nội tại (vòng trong) chưa ổn định, khiến quá trình tune không hội tụ. Quy trình chuẩn:
 
1. **Khoá vòng ngoài** (dùng `Manual Switch`), chỉ giữ vòng `PID Angle` hoạt động với setpoint góc bằng hằng số (`Constant`/`Step`) → tune cho tới khi robot giữ thăng bằng tại chỗ, đáp ứng nhanh, ít vọt lố.
2. **Mở khoá vòng ngoài**, giữ nguyên `PID Angle` vừa tune được, chỉ chỉnh `PID Pos` để robot bám theo vị trí/vận tốc mong muốn mà không làm mất ổn định góc.
3. Dùng khối `Step` làm nhiễu bậc thang ở từng giai đoạn để kiểm chứng độ bền vững (robustness) — tương tự cách dùng `DiscretePulseGenerator` để kiểm tra kháng nhiễu trong các mô hình con lắc ngược cổ điển.
---
 
## 5. Kết Quả Mô Phỏng
 
*(Chèn ảnh chụp `Scope`/`Main Scope` và video mô phỏng của bạn vào phần này — ví dụ: đáp ứng góc nghiêng $\theta$, vị trí $x$, và tín hiệu điều khiển khi bật/tắt từng vòng qua `Manual Switch`.)*
 
```
![Đáp ứng góc nghiêng và vị trí khi bật Cascade PID](/images/cascade-pid-robot/plot_cascade_pid.jpg)
```
 
Các điểm cần đọc trên đồ thị:
 
1. **Góc nghiêng $\theta$:** thời gian ổn định, độ vọt lố sau khi bật vòng trong.
2. **Vị trí $x$ / vận tốc:** robot có bám đúng setpoint từ `PID Pos` hay không, có còn hiện tượng trôi (drift) như hệ chỉ dùng một vòng PID hay không.
3. **Tín hiệu điều khiển đầu ra:** có bị bão hoà liên tục (chạm giới hạn ±0.2 của vòng ngoài) hay không — nếu có, cần giảm độ lợi `PID Pos` hoặc nới giới hạn output.
---
 
## 6. Kết Luận & Hướng Phát Triển
 
Kiến trúc Cascade PID (Position → Angle) giải quyết được vấn đề trôi vị trí vốn có của một vòng PID góc đơn lẻ, đồng thời giữ được sự đơn giản trong triển khai so với các bộ điều khiển trạng thái đầy đủ.
 
**Hướng phát triển tiếp theo:**
 
1. **Kalman filter / bộ lọc bù trên phản hồi góc** để giảm nhiễu đo từ IMU trước khi đưa vào vòng trong.
2. **LQR toàn trạng thái** $\mathbf{x} = [x, \dot{x}, \theta, \dot{\theta}]^T$ thay thế cấu trúc cascade khi cần tối ưu đồng thời nhiều mục tiêu.
3. **Đưa toàn bộ hệ số PID lên có thể chỉnh qua Serial** trên firmware thực tế (ESP32 + SimpleFOC) để so sánh trực tiếp với kết quả mô phỏng Simulink mà không cần nạp lại firmware mỗi lần tune.
---
 
**Tags:** cascade-pid, self-balancing-robot, control-systems, simulink, matlab, esp32, simplefoc
 
