# UR3 LLM Control

Hệ thống mô phỏng robot UR3/UR3e trên ROS 2 Humble. Người dùng gửi câu lệnh tiếng Việt hoặc tiếng Anh; LLM chuyển câu lệnh thành thao tác lấy và đặt khối, sau đó MoveIt 2 lập kế hoạch và điều khiển robot trong Gazebo.

## 1. Chuẩn bị

Cần cài ROS 2 Humble, các gói `ur_simulation_gz`, `ur_moveit_config` và dịch vụ 9Router đang chạy. Trên 9Router, hãy kết nối ít nhất một nhà cung cấp LLM và tạo API key.

## 2. Biên dịch package

Mở terminal tại workspace ROS 2 và chạy một lần sau khi tải mã nguồn hoặc sửa code:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_llm_control
source install/setup.bash
```

## 3. Khởi động hệ thống

Mở **Terminal 1**. 

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=24
export NINE_ROUTER_BASE_URL="http://localhost:20128/v1"
export NINE_ROUTER_MODEL="oc/muse-spark-1.3-contributor-free"
export NINE_ROUTER_API_KEY="sk-3d97f003a0bdf37e-d5t80q-3be91b02"

ros2 launch ur3_llm_control llm_robot.launch.py

```

Lệnh này khởi động Gazebo, RViz, MoveIt, LLM planner và skill executor. Chờ robot về tư thế ban đầu trước khi gửi lệnh. Nếu dùng UR3 thay vì UR3e, chạy:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py ur_type:=ur3
```

Để chỉ kiểm tra việc lập kế hoạch, không cho robot di chuyển, thêm `execute_motion:=false` vào lệnh launch.

## 4. Gửi lệnh cho robot

Mở **Terminal 2** trong khi Terminal 1 vẫn đang chạy:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=24
```

Gửi một câu lệnh, ví dụ các câu theo phân công cá nhân (A → vàng, B → xanh dương, C → đỏ):

```bash
ros2 run ur3_llm_control command “Move the yellow cube to zone A.”
ros2 run ur3_llm_control command “Đưa khối màu vàng vào vùng A.”
 
ros2 run ur3_llm_control command “Pick up the blue cube and place it in zone B.”
ros2 run ur3_llm_control command “Lấy khối màu xanh dương và đặt vào vùng B.”

ros2 run ur3_llm_control command “Please put the red cube in zone C.”
ros2 run ur3_llm_control command “Vui lòng đặt khối màu đỏ vào vùng C.”
```

Mỗi lần chỉ gửi một lệnh và chờ lệnh trước chạy xong. Terminal sẽ báo `RESULT: SUCCESS` khi hoàn tất hoặc `RESULT: FAILED` khi có lỗi. Có thể xem trạng thái chi tiết bằng:

```bash
ros2 topic echo /task_status
```

### Lệnh nâng cao: sắp xếp toàn bộ vật thể

Hệ thống hỗ trợ một lệnh tạo kế hoạch cho nhiều vật thể. Theo cấu hình sinh viên hiện tại, robot sẽ đưa khối vàng vào A, khối xanh dương vào B và khối đỏ vào C, sau đó trở về vị trí home:

```bash
ros2 run ur3_llm_control command "Arrange all objects according to student ID = 33."
ros2 run ur3_llm_control command "Sắp xếp tất cả khối theo student ID = 33."
```

Kế hoạch gồm ba cặp `pick`/`place` và một `home()` duy nhất ở cuối. Robot dừng 2 giây sau mỗi lần đặt khối, trước khi gắp khối kế tiếp, và không về home giữa chừng. Các vùng đích trong scene khởi tạo đều trống; khi cần sắp xếp lại sau một lệnh trước đó, hãy khởi động lại scene để đưa các khối về vị trí ban đầu.

## 5. Đối tượng và vùng hợp lệ

| Tên hiển thị | Tên hệ thống |
| --- | --- |
| Khối đỏ | `red_cube` |
| Khối vàng | `yellow_cube` |
| Khối xanh dương | `blue_cube` |
| Vùng A, B, C | `zone_a`, `zone_b`, `zone_c` |

Vị trí khối, vị trí vùng, chiều cao tiếp cận, vận tốc và gia tốc được cấu hình trong `config/scene.yaml`. Thông tin sinh viên và bài phân công nằm trong `config/student.yaml`.
