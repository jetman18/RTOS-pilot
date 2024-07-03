# RTOS-pilot
Mã nguồn và mạch điều khiển bay cho UAV cánh cố định sử dụng hệ điều hành thời gian thực FreeRTOS. Mạch sử dụng vi điều khiển chính STM32F407vet6 và vi điều khiển phụ STM32F103c8t6, vi điều khiển phụ được bổ sung nhằm xử lý tình huống khi mạch chính gặp lỗi treo hoặc các lỗi phần cứng khác. Mạch nguyên lý và PCB được thiết kế trên phần mềm Proteus.

<img align="right" src="./images/fc1.jpg" alt="img-name" width="350" height="230">
<img align="left" src="./images/3dmode.png" alt="img-name" width="350" height="230"> 

## Phần cứng
#### Cảm biến và các linh kiện chính
- Cảm biến gia tốc, vận tốc góc 6 trục MPU6050 
- Cảm biến áp suất MS5611 sử dụng để xác định độ cao
- Cảm biến la bàn HMC5883 
- IC nhớ EPPROM AT24C256 lưu giữ các giá trị cấu hình
- Thẻ nhớ sử dụng để lưu dữ liệu bay
- STM32F407vet6 VĐK chính
- STM32F103c8t6 VĐK phụ
- Module GPS M10 và NEO 7M
- Còi chíp báo dộng
#### Các cổng giao tiếp
- 3 Uart
- 1 I2C
- 8 GPIO
- 6 PWM out
- 1 cổng USB sử dụng để nạp chương trình


## Phần mềm
Sử dụng hệ điều hành RTOS với 3 Task Realtime chính 
- Task 1: tần số 500HZ (do sử dụng giao thức I2C nên tốc độ giao tiếp chậm, có thể cải tiến sử dụng các cảm biến sử dụng SPI như MPU6000 để có thể tăng tốc độ đọc dữ liệu) task đọc và lọc dữ liệu cảm biến MPU6050, với dữ liệu gia tốc sử dụng bộ lọc thông thấp, với vận tốc góc sử dụng bộ lọc trung bình
- Task 2: Tần số 100Hz nhiệm vụ ước lượng góc trạng thái Roll, Pitch, Yaw sử dụng thuật toán Mahony filter kết hợp các phép đo từ cảm biến gia tốc, vận tốc góc và la bàn
- Task 3: Tần số <= 100 Hz, có nhiệm vụ lưu dữ liệu bay vào thẻ nhớ nhằm sử dụng để phân tích dữ liệu cũng như xác định lỗi khi UAV bị rơi 

Sơ đồ hệ thống điều khiển

<img align="center" src="./images/block.png" alt="img-name" width="900" height="500">

### Tổ chức Files thư mục chính
  Core  ./  Driver $~~~~~~~$    thư viện giao tiếp với các cảm biến   
$~~~~~~~$ ./ epprom $~~~~~$    thư viện giao tiếp với epprom AT24c
$~~~~~~~$ ./ flight $~~~~~~~~~$    thư viện xử lý chính cho UAV 
$~~~~~~~$ ./ HIL $~~~~~~~~~~~$    thư viện mô phỏng HIL $~~$(loading ...) 
$~~~~~~~$ ./ inc $~~~~~~~~~~~~$    thư viện chứa Header file do cubemx gen 
$~~~~~~~$ ./ Lib $~~~~~~~~~~~~$    thư viện chung
$~~~~~~~$ ./ mavlink $~~~~~$    thư viện giao tiếp mavlink 
$~~~~~~~$ ./ Src $~~~~~~~~~~~$    thư viện chứa Source file do cubemx gen  
$~~~~~~~$ ./ Startup


## Thử nghiệm
Cấu hình UAV thử nghiệm:
- Sải cánh 1.2 m
- Trọng lượng 0.85 kg
- Vận tốc tối đa 110 km/h
- Thời gian bay 10 - 20 phút

Cấu hình đồ điện:

- Motor sunnysky 2216 2400 kv 
- Pin lipo 3s 2200Mah
- Bộ điều khiển Flysky Fs-i6
- Bộ điều tốc 40A

Lắp đặt mạch và các linh kiện lên UAV


<img align="center" src="./images/datmach.jpg" alt="img-name" width="900" height="600">


UAV dạng flying wing 

<img  src="./images/fpvv.jpg" alt="img-name" width="900" height="600"> 

### Dữ liệu bay thủ công
##### Quỹ đạo bay từ GPS
<img  src="./images/flight_trajectory.jpg" alt="img-name" width="400" height="300"> 

##### Altitude baro vs gps
<img  src="./images/alt_baro_vs_gps.jpg" alt="img-name" width="700" height="330"> 

##### Roll && pitch angle
<img  src="./images/roll_pitch_data.jpg" alt="img-name" width="700" height="330"> 

##### Ground speed from gps
<img  src="./images/g_speed.jpg" alt="img-name" width="700" height="330"> 


## Attitude && PID controller

##### Roll axis 

[//]:<p align="center">
<img src="./images/roll.png" width="700" height="350" />
</p>

##### Pitch axis 

[//]:<p align="center">
<img src="./images/pitch1.png" width="700" height="350" />
</p>



