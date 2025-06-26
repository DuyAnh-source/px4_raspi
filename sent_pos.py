from pymavlink import mavutil
import time
import threading
import px4_utils as px4

# master = mavutil.mavlink_connection('COM3', baud=2000000)
master = mavutil.mavlink_connection('/dev/ACM0', baud=2000000)
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
    px4.send_movement_pos_ned(origin_ned = origin_ned,x = 0,y = 0,altitude = 1)
    # send_movement_command(0,0,1)
    time.sleep(0.05)

px4.offboard_mode()
px4.arm()
print ("✅ Chuyển sang OFFBOARD mode và đã ARM")

stop_event = threading.Event()
def thread_send_commands():
    while not stop_event.is_set():
        px4.send_movement_pos_ned(origin_ned = origin_ned,x = 0,y = 0,altitude = 1)
        time.sleep(0.05)
def thread_read_msg():
    while not stop_event.is_set():
        msg = master.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=True, timeout=2)
        if msg:
            print(msg)

t1 = threading.Thread(target=thread_send_commands, name="send_commands")
t2 = threading.Thread(target=thread_read_msg, name="read_msg")

t1.start()
t2.start()

# Cho bay 5s rồi dừng
time.sleep(10) 

stop_event.set()
px4.land_mode()

t1.join()
t2.join()

print("✅ Dừng tất cả threads và kết thúc chương trình.")
