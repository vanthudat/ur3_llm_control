# UR3 LLM Control – Bài 03, student ID = 33

ROS 2 Humble + Gazebo Fortress + MoveIt 2. Phân công cá nhân: **Zone A → vàng, Zone B → xanh dương, Zone C → đỏ**.

Môi trường gồm một UR3/UR3e, một gripper, camera RGB-D trên bàn và camera RGB-D cận cảnh gắn trên gripper, bàn thao tác, ba zone và năm block: đỏ, vàng, xanh dương, xanh lá, tím. Hai block xanh lá và tím không có zone đích riêng trong phân công ID 33; hệ thống vẫn nhận diện chúng và chuyển ra vị trí tạm khi chúng chiếm chỗ.

```text
Câu lệnh + quan sát camera → LLM Planner → Structured Plan
→ Plan Validator + logic dọn zone → Robot Skills → MoveIt 2 → UR3/UR3e + Gripper
```

## Chuẩn bị và biên dịch

Cần ROS 2 Humble, `ur_simulation_gz`, `ur_moveit_config`, `ros_gz_bridge`, Python OpenCV và NumPy. Dịch vụ 9Router phải có model đang hoạt động và API key. Camera RGB-D cần Gazebo render được bằng Ogre2, kể cả khi tắt GUI.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_llm_control
source install/setup.bash
```

## Khởi động

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=24
export NINE_ROUTER_BASE_URL="http://localhost:20128/v1"
export NINE_ROUTER_MODEL="YOUR_AVAILABLE_MODEL_ID"
read -rsp "Nhap 9Router API key: " NINE_ROUTER_API_KEY
export NINE_ROUTER_API_KEY
echo
ros2 launch ur3_llm_control llm_robot.launch.py
```

Mặc định UR3e; thêm `ur_type:=ur3` để dùng UR3. Mỗi lần khởi chạy Gazebo, `green_cube` và `purple_cube` được xáo vị trí ngẫu nhiên: một block phụ ở một zone ngẫu nhiên, block còn lại ở zone khác hoặc vị trí khác trên bàn. Terminal in vị trí khởi tạo để tiện đối chiếu; planner vẫn chỉ dùng camera để lập kế hoạch. Dùng `randomize_obstacles:=false` để giữ bố cục SDF cố định. Chờ executor báo ready và camera nhận diện đủ năm block trước khi gửi lệnh. `execute_motion:=false` từ chối thực thi task; chế độ này không mô phỏng kết quả gắp–thả hoặc báo task thành công.

### Mở RViz ở terminal riêng

Terminal 1 chạy Gazebo, MoveIt và các node điều khiển (mặc định không mở RViz):

```bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

Terminal 2 mở RViz, cùng ROS domain với Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=24
ros2 launch ur3_llm_control rviz.launch.py
```

Nếu Terminal 1 dùng `ur_type:=ur3`, Terminal 2 cũng thêm `ur_type:=ur3`. RViz mặc định dùng profile nhẹ chỉ dựng robot và ảnh camera. Khi cần bảng lập kế hoạch tương tác MoveIt, chọn profile đầy đủ bằng `rviz_config:=view_moveit.rviz`.

Profile nhẹ hiển thị robot/gripper từ `/robot_description` và ảnh trực tiếp từ Wrist Camera. Profile MoveIt bổ sung bảng MotionPlanning. Cả hai dùng Fixed Frame `world`; robot chuyển động theo TF và `/joint_states` do hệ thống ở Terminal 1 cung cấp, nên Terminal 1 phải chạy trước khi mở RViz riêng. Backend MoveIt vẫn chạy ở Terminal 1 kể cả khi dùng profile nhẹ.

## Demo cá nhân hóa

Ba block theo phân công ban đầu nằm ngoài zone; `green_cube` và `purple_cube` được đặt ngẫu nhiên khi khởi động, trong đó luôn có một block phụ chiếm một zone ngẫu nhiên. Màu block nào chiếm zone và zone bị chiếm thay đổi giữa các lần chạy. Tất cả block là vật thể động, có khối lượng, quán tính và ma sát. Gripper giữ vật bằng tiếp xúc vật lý, không gọi dịch vụ sửa pose.

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=24
ros2 topic echo /environment_state --once --full-length
ros2 run ur3_llm_control command "Put the blue cube in Zone B." --timeout 600
```

Planner nhận yêu cầu ID 33, đọc ảnh camera để tìm block đang chiếm Zone B (có thể là xanh lá hoặc tím), chọn vị trí tạm còn trống, chuyển vật cản khỏi B rồi gắp xanh dương đặt vào B. Kế hoạch và kết quả từng skill xuất trên `/task_status`.

```bash
ros2 run ur3_llm_control command "Arrange all objects according to student ID = 33." --timeout 900
```

Lệnh này sắp xếp vàng → A, xanh dương → B, đỏ → C. Xanh lá và tím là vật cản được chuyển khi cần. Block đã nằm đúng vị trí được bỏ qua. Gửi từng lệnh và chờ hoàn tất; không cần khởi động lại world sau mỗi lệnh thành công.

Mặc định, terminal lệnh chỉ hiện camera check, kế hoạch đã xác thực, trạng thái từng `pick/place/home` và kết quả cuối để log gọn khi chụp báo cáo. Dùng `--verbose` sau command nếu cần xem subskill và thông tin planning chi tiết:

```bash
ros2 run ur3_llm_control command "Arrange all objects according to student ID = 33." --timeout 900 --verbose
```

## Camera, skill và validator

Camera RGB-D chính đặt cố định phía trên tâm bàn tại `[0.25, 0, 2.0]`, nhìn thẳng xuống với FOV ngang 75°. Ở độ cao này, vùng phủ hình học xấp xỉ `1.96 × 1.47 m`, rộng hơn mặt bàn `0.80 × 0.65 m`; thân camera được đỡ bằng boom chéo từ ngoài mép bàn. Camera cổ tay gắn bên hông gripper và hướng vào tâm khe kẹp. `camera_state` dùng ảnh camera bàn, đồng bộ RGB/depth, phân đoạn năm màu, dùng intrinsics từ `CameraInfo` và depth để chuyển điểm ảnh sang tọa độ world. Ảnh RGB-D cổ tay cũng được đồng bộ; hệ thống chỉ chấp nhận đúng màu trong vùng ảnh gần và khoảng cách `0.04–0.16 m`, rồi yêu cầu hai frame mới liên tiếp sau khi nâng và khi đang mang vật. Đây là xác nhận cận cảnh, không định vị 3D, nên không cần hand-eye calibration. Không đọc pose model Gazebo hoặc tọa độ block từ YAML.

| Topic | Nội dung |
| --- | --- |
| `/table_camera/image` | Ảnh RGB |
| `/table_camera/depth_image` | Depth `32FC1` |
| `/table_camera/camera_info` | Thông số camera |
| `/table_camera/detections` | Ảnh có kết quả nhận diện để đưa vào video |
| `/wrist_camera/image` | Ảnh RGB cận cảnh từ camera gắn trên gripper, hiển thị trong RViz |
| `/wrist_camera/depth_image` | Depth từ camera cổ tay |
| `/wrist_camera/camera_info` | Thông số camera cổ tay |
| `/wrist_camera/observation` | Màu block, pixel và depth cận cảnh dùng để xác nhận gắp |
| `/environment_state` | JSON tọa độ quan sát và trạng thái zone |
| `/validated_plan` | Kế hoạch skill đã kiểm tra |
| `/task_status` | Camera check, bước dọn chỗ, kế hoạch, tiến độ và kết quả |

LLM chỉ hiểu yêu cầu, chọn skill, xác định tên block/zone và thứ tự hành động; không sinh joint trajectory hoặc lệnh điều khiển joint. Logic môi trường chọn một trong các vị trí tạm ứng viên sau khi kiểm tra khoảng trống từ camera; vị trí tạm phải cách các vật khác ít nhất 12 cm để chừa sai số nhận diện. Python validator kiểm tra schema và giả lập trạng thái chiếm chỗ của toàn bộ kế hoạch; executor C++ kiểm tra lại trước mỗi lần gắp. Terminal hiển thị `CAMERA CHECK` cho vật đang chiếm zone, `CLEARANCE PLAN` cho cặp pick/place dọn vật cản, kế hoạch skill đã mở rộng và `EXECUTION PLAN`. Nếu camera mới cho thấy chỗ tạm đã chọn sát vật khác hoặc bị chiếm, executor tìm chỗ tạm khác, cập nhật kế hoạch đang chạy và in `EXECUTION PLAN UPDATE`.

Mỗi `pick` kiểm tra lại đích và yêu cầu camera mới, đủ năm block; mỗi `place` yêu cầu các block còn lại nhìn thấy được và đích còn trống. Camera phải xác nhận ba frame liên tiếp rằng vật đã nâng lên hoặc đã đặt đúng vị trí. Sau `home`, camera xác nhận lại vị trí cuối của các block đã di chuyển trước khi báo `TASK SUCCESS`. Nếu thất bại, task dừng để người vận hành kiểm tra. Khi robot còn giữ vật sau lỗi, cần xử lý trạng thái đang giữ trước khi nhận task mới; hệ thống chưa có skill tự phục hồi.

`zone_status` trong `/environment_state` gồm `free`, `occupied` hoặc `unknown`. Khi thiếu block do che khuất, hệ thống báo `unknown`, không suy ra zone trống chỉ từ việc không thấy vật.

Độ mới của camera được kiểm tra bằng thời gian nhận ảnh trên máy (`observed_at`) và thời gian monotonic tại planner/executor, độc lập với thứ tự nhận `/clock`. Timestamp Gazebo `stamp` được giữ để đồng bộ RGB/depth. Mỗi frame có `stream_id` và `frame_id`; dữ liệu lặp hoặc đến sai thứ tự không được tính là quan sát mới. Camera ngừng gửi frame hoặc ảnh đã nằm chờ quá hai giây vẫn bị từ chối. Khi nâng cấp cơ chế timestamp, cần build lại rồi khởi động lại toàn bộ launch để camera, planner và executor dùng cùng định dạng dữ liệu.

MoveIt cập nhật collision objects từ camera, kiểm tra va chạm và lập trajectory bên trong robot skills. Collision object của vật gắp được attach vào gripper trong MoveIt; Gazebo vẫn dùng vật lý tiếp xúc riêng. Vị trí tạm cần cả khoảng trống và kế hoạch MoveIt hợp lệ; nếu không có đường chuyển động hợp lệ, executor báo thất bại.

Trước khi tiếp cận vật, skill gắp lập kế hoạch tới điểm phía trên vật, xuống gắp và chuyển tới phía trên đích. Hệ thống thử bốn hướng quay gripper quanh trục đứng và ba độ cao tiếp cận phù hợp với block vuông. Chỉ thực thi tiếp cận khi các đường này hợp lệ; đường xuống được tính lại từ feedback joint thực tế. Preflight chuyển vật là kiểm tra sơ bộ với gripper mở; đường thực thi vẫn phải kiểm tra lại sau khi attach collision object của vật. Log `grasp_preflight`, `descend`, `joint_jump`, `trajectory_bounds` và `PLANNING_DETAIL` giúp xác định bước bị từ chối.

Nếu Cartesian từ MoveIt trả về đường thiếu hoặc đổi nhánh IK, executor thử giải IK liên tục từng điểm, lấy nghiệm trước làm seed và áp dụng consistency limit ngay trong solver. Mỗi nghiệm chỉ được thay đổi tối đa 0.10 rad/joint; các trạng thái joint trung gian được kiểm tra giới hạn và va chạm qua MoveIt. Đường hoàn chỉnh vẫn qua kiểm tra tổng quãng quay và time parameterization trước khi chạy. `PLANNING_DETAIL` báo điểm IK/va chạm thất bại hoặc đường seeded Cartesian được chấp nhận. Cách này không tăng giới hạn nhảy joint hoặc bỏ kiểm tra va chạm; không bảo đảm mọi cấu hình vật đều có đường gắp hợp lệ.

Khi đang giữ vật, robot dùng đường Cartesian giữ hướng gripper, từ điểm đã nâng tới điểm phía trên zone; không chuyển sang RRT nếu đường thẳng không hợp lệ. Cartesian thử bước lấy mẫu 5 mm, 2.5 mm, 1.25 mm và yêu cầu toàn bộ đường hợp lệ. Bộ lọc nhảy joint tương đối được thay bằng giới hạn tuyệt đối 0.10 rad mỗi bước để tránh từ chối sai đường ngắn và chặn đổi nhánh IK. IK khi gripper trống ưu tiên trạng thái joint hiện tại để giảm đổi nhánh khuỷu/cổ tay. Kế hoạch bị từ chối nếu có bước nhảy joint hoặc tổng quãng quay vượt quá mức đi vòng cho phép. Tốc độ/gia tốc Cartesian được giảm xuống 0.05/0.03 để giảm giật vật.

Hai joint gripper dùng command interface `effort` và PID của `gripper_controller` để duy trì lực ép khi tiếp xúc. Với block cạnh 40 mm, lệnh đóng 18 mm mỗi ngón tạo mục tiêu khe 36 mm, nhỏ hơn block để PID tiếp tục tạo lực giữ. Không coi goal gripper bị abort là gắp thành công. Camera theo dõi vật so với vị trí gripper trong suốt các chuyển động gắp/nâng/chuyển/hạ; robot dừng nếu vật lệch trên 40 mm ở hai frame liên tiếp hoặc mất xác nhận hơn 1.5 giây. Trước khi hạ và mở gripper, camera phải xác nhận vật vẫn nằm trên zone đích. Khi thay controller position sang effort cần dừng launch, build lại và khởi động lại Gazebo để nạp lại cả URDF lẫn controller.

Trước khi chuyển tới đích, executor yêu cầu camera quan sát các block còn lại. Khi tới phía trên zone, robot xác nhận vật vẫn được giữ rồi hạ thẳng; không có vòng nâng thêm 4/8/12 cm. Trong một lượt đặt, các block khác được coi là đứng yên: nếu block ở xa đích bị tay máy che, collision object vẫn được giữ tại vị trí camera đã đo trước lượt chuyển, tối đa 30 giây. Block không thấy mà vị trí đã đo cách đích dưới 135 mm vẫn làm bước kiểm tra bị từ chối; mọi block thấy trong ảnh mới được cập nhật. Sau khi thả, camera phải xác nhận vật đã đặt đúng đích; việc kiểm tra toàn cảnh thực hiện lại ở các bước tiếp theo và cuối kế hoạch sau `home()`. Logic này dành cho bàn mô phỏng có các block thụ động, không có tác nhân khác di chuyển vật trong lượt đặt. Các lỗi được phân biệt bằng `verify_grasp`, `verify_scene`, `verify_destination`. Nếu không thể xác nhận, robot giữ gripper đóng và báo thất bại.

Điểm mở gripper cách mặt tiếp xúc với bàn 2 mm (`placement_clearance`), tránh collision object chạm chính xác biên bàn khi lập Cartesian path. Vật ổn định xuống bàn bằng trọng lực sau khi thả; camera xác nhận vị trí thực tế.

`config/student.yaml` giữ mã sinh viên đầy đủ `23020733`, `assignment_id: 33` và phân công cá nhân: vàng→A, xanh dương→B, đỏ→C. Với lệnh sắp xếp có nêu ID 33, planner giữ đúng ba đích này. Logic môi trường dùng camera phát hiện vật đang chiếm đích rồi chèn bước chuyển vật cản sang vị trí tạm ngay trước cặp gắp/đặt của block được phân công. Nếu một block phụ đang ở B, kế hoạch là gắp vàng→A, chuyển block đang chiếm B sang vị trí tạm, gắp xanh dương→B, gắp đỏ→C, rồi về home (bỏ qua các block vốn đã ở đúng đích). Nếu B trống, không cần dọn B. Màu xanh lá và tím không có đích phân công riêng. `config/scene.yaml` chỉ cấu hình robot, bàn và zone. Khi thay world, camera hoặc kích thước vật, phải cập nhật hiệu chuẩn trong `perception.py`, tọa độ zone/vị trí tạm trong `environment.py` và executor tương ứng.

## Giải thích logic và bài nộp

Với yêu cầu đặt xanh dương vào B, nếu camera thấy một block phụ chiếm B, hệ thống thực hiện:

```text
Camera xác nhận green_cube hoặc purple_cube ở zone_b
→ tìm vị trí tạm đủ khoảng trống và kiểm tra IK
→ pick(blocker_cube) → place(blocker_cube, temp_i)
→ camera cập nhật trạng thái, kiểm tra lại zone_b
→ pick(blue_cube) → place(blue_cube, zone_b)
→ home → camera xác nhận kết quả cuối
```

Vị trí khởi tạo trong SDF chỉ tạo bố cục mô phỏng. Camera cung cấp vị trí block cho lập kế hoạch và collision scene. Các vị trí tạm được lấy từ lưới ứng viên phủ mặt bàn; một vị trí chỉ được chọn khi quan sát xác nhận còn trống, đủ khoảng cách với vật khác và MoveIt có nghiệm IK hợp lệ. MoveIt tiếp tục kiểm tra đường chuyển động thực tế khi thực thi skill.

Khi chuyển sang vật tiếp theo trong lệnh ID 33, executor chờ toàn cảnh camera đầy đủ. Nếu tay máy trống vẫn che camera sau khi rút khỏi zone trước, robot về `home()` một lần rồi chờ ảnh mới chụp sau chuyển động trước khi kiểm tra lượt gắp tiếp theo. Đây là bước phục hồi quan sát giữa hai lượt gắp, không phải nâng vật nhiều lần trước khi đặt. Log `camera_observation_home` và `camera_full_scene` thể hiện bước này; `transfer_destination_occupied` và `transfer_destination_ik` phân biệt lỗi đích bị chiếm với lỗi IK.

Các thành phần chính: `camera_state.py` nhận và đồng bộ ảnh; `perception.py` nhận diện màu và dựng tọa độ 3D; `environment.py` xử lý vật cản; `task_validator.py` kiểm tra schema; `llm_planner.py` gọi LLM; `skill_executor.cpp` kiểm tra kế hoạch trước thực thi; `robot_skills.cpp` điều khiển MoveIt/gripper và xác nhận trạng thái bằng camera.

Video và báo cáo cần thể hiện dữ liệu camera, kế hoạch dọn zone, thao tác vật lý và trạng thái cuối; yêu cầu nộp GitHub và các tiêu chí Bài 02 vẫn giữ nguyên. Phần giải thích ở đây mô tả thiết kế đã triển khai; video demo và URL repository cần bổ sung khi nộp bài.
