from pymavlink import mavutil
import time
import threading
import px4_utils as px4
import log_csv as log
import math
# master = mavutil.mavlink_connection('COM3', baud=2000000)
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=2000000)
master.wait_heartbeat()
print("✅ Kết nối thành công với PX4")
# === Hàm lấy time_boot_ms đã đồng bộ ===
px4.init_globals(master)
px4.get_time_px4()
print(f"🕒 Đồng bộ thành công: time_boot_ms FC = {px4.t0_fc} ms")

origin_msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
if origin_msg is None:
    print("❌ Không nhận được vị trí từ PX4.")
    exit(1)

origin_ned = {
    'x': origin_msg.x,
    'y': origin_msg.y,
    'z': origin_msg.z
}
print(f"✅ Gốc tọa độ (NED): {origin_ned}")

print("🚀 Gửi setpoint để PX4 chấp nhận OFFBOARD...")
   
def send_movement_command(x = 0,y = 0,altitude = 0):
    master.mav.set_position_target_local_ned_send(
    px4.get_synced_time_boot_ms(),
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111111000,  # chỉ x, y, z
    origin_ned['x'] + x, origin_ned['y'] + y, origin_ned['z'] - altitude,  # Z là độ cao so với gốc
    0, 0, 0, 0, 0, 0,
    0, 0
)

for _ in range(20):  # gửi trong ~1s 
    send_movement_command(0,0,1)
    time.sleep(0.05)

px4.offboard_mode()
px4.arm()
print ("✅ Chuyển sang OFFBOARD mode và đã ARM")

stop_event = threading.Event()
def thread_send_commands():
    while not stop_event.is_set():
        send_movement_command(0,0,1.5)
        time.sleep(0.05)

log.init_log(px4.t0_pc)  # Khởi tạo file log
def task2():  
    while not stop_event.is_set():
        msg1 = px4.get_servo_output_raw()
        msg2 = px4.get_attitude() 
        msg3 = px4.get_local_position_ned()
        if msg1 is not None and msg2 is not None and msg3 is not None:
            servo1_raw = msg1.servo1_raw
            servo2_raw = msg1.servo2_raw
            servo3_raw = msg1.servo3_raw
            servo4_raw = msg1.servo4_raw    

            roll_feedback = msg2.roll * 180 / math.pi
            pitch_feedback = msg2.pitch * 180 / math.pi
            yaw_feedback = msg2.yaw * 180 / math.pi

            x_local = msg3.x - origin_ned['x']
            y_local = msg3.y - origin_ned['y']
            z_local = msg3.z - origin_ned['z']

            log.log_data(x_local, y_local, z_local,
                         roll_feedback, pitch_feedback, yaw_feedback,
                         servo1_raw, servo2_raw, servo3_raw, servo4_raw)
        time.sleep(0.1)

t1 = threading.Thread(target=thread_send_commands)
t2 = threading.Thread(target=task2)

t1.start()
t2.start()
  
time.sleep(7)
stop_event.set()
t1.join()
t2.join()

print("✅ Dừng tất cả threads và kết thúc chương trình.")
