import csv
import os
import time
import math

log_file = "log.csv"
t0_pc = 0
def init_log(t0 = 0):
    global log_file
    global t0_pc
    t0_pc = t0 if t0 > 0 else time.time()
    if not os.path.exists(log_file):
        with open(log_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x_local', 'y_local', 'z_local', 
                             'roll_feedback', 'pitch_feedback', 'yaw_feedback', 
                             'servo1_raw','servo2_raw','servo3_raw','servo4_raw'])
            
def log_data(x,y,z,
             roll_feedback, pitch_feedback, yaw_feedback, servo1_raw, servo2_raw, servo3_raw, servo4_raw):
    global log_file
    timestamp = time.time() - t0_pc
    with open(log_file, mode='a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, x, y, z,
                         roll_feedback, pitch_feedback, yaw_feedback,
                         servo1_raw, servo2_raw, servo3_raw, servo4_raw])