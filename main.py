from pyPS4Controller.controller import Controller
import serial
import time
import threading

# ==========================================
# KONFIGURASI SERIAL & CONTROLLER
# ==========================================
SERIAL_PORT = '/dev/ttyACM0'  # pyPS4Controller umumnya berjalan di Linux/Pi
BAUD_RATE = 115200
CONTROLLER_INTERFACE = "/dev/input/js0"

# ==========================================
# KONFIGURASI KECEPATAN & DURASI
# ==========================================
MAX_SLIDER1_SPEED = 150      
SLIDER1_DURATION_SEC = 2.5   # Dikonversi ke detik untuk time.time()
MAX_SLIDER2_SPEED = 255      
SLIDER2_DURATION_SEC = 1.2   

UP_SPEED_LEFT  = 255 
UP_SPEED_RIGHT = 217 
DOWN_SPEED_LEFT  = 248
DOWN_SPEED_RIGHT = 217 

DEFAULT_MECANUM_SPEED = 100 
BOOST_MECANUM_SPEED = 170

# ==========================================
# KONFIGURASI STEP SERVO 
# ==========================================
SERVO1_STEPS = [30, 120, 190] # Index 0, 1, 2
SERVO2_STEPS = [85, 85, 110] 

# ==========================================
# FUNGSI BANTUAN
# ==========================================
def map_value(value, min_in, max_in, min_out, max_out):
    return int((value - min_in) * (max_out - min_out) / (max_in - min_in) + min_out)

# ==========================================
# KELAS CONTROLLER PS4 (EVENT DRIVEN)
# ==========================================
class RobotController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        
        # --- State Analog (-32767 hingga 32767) ---
        self.lx = 0
        self.ly = 0
        self.rx = 0

        # --- State Modifiers ---
        self.is_l2_pressed = False
        self.is_r2_pressed = False

        # --- State Mekanisme Lift ---
        self.btn_triangle = False
        self.btn_x = False

        # --- State Relay ---
        self.relay1 = 0
        self.relay2 = 0
        self.relay3 = 0
        self.relay4 = 0

        # --- State Servo ---
        self.servo1_idx = 1
        self.servo2_idx = 1

        # --- State Slider ---
        self.s1_moving, self.s1_is_out, self.s1_start_time = False, False, 0
        self.s2_moving, self.s2_is_out, self.s2_start_time = False, False, 0

    # --- CALLBACKS ANALOG KIRI (Kinematika XY) ---
    def on_L3_up(self, value): self.ly = value
    def on_L3_down(self, value): self.ly = value
    def on_L3_y_at_rest(self): self.ly = 0
    
    def on_L3_left(self, value): self.lx = value
    def on_L3_right(self, value): self.lx = value
    def on_L3_x_at_rest(self): self.lx = 0

    # --- CALLBACKS ANALOG KANAN (Rotasi) ---
    def on_R3_left(self, value): self.rx = value
    def on_R3_right(self, value): self.rx = value
    def on_R3_x_at_rest(self): self.rx = 0

    # --- CALLBACKS L2 & R2 ---
    def on_L2_press(self, value): self.is_l2_pressed = True
    def on_L2_release(self): self.is_l2_pressed = False
    
    def on_R2_press(self, value): self.is_r2_pressed = True
    def on_R2_release(self): self.is_r2_pressed = False

    # --- CALLBACKS ACTION BUTTONS (Slider, Lift, Servo) ---
    def on_square_press(self):
        if self.is_l2_pressed:
            self.servo1_idx, self.servo2_idx = 0, 0
            print(f"-> Servo: Posisi 1 | S1: {SERVO1_STEPS[0]}°, S2: {SERVO2_STEPS[0]}°")
        else:
            if not self.s2_moving:
                self.s2_moving, self.s2_start_time = True, time.time()

    def on_triangle_press(self):
        if self.is_l2_pressed:
            self.servo1_idx, self.servo2_idx = 1, 1
            print(f"-> Servo: Posisi 2 | S1: {SERVO1_STEPS[1]}°, S2: {SERVO2_STEPS[1]}°")
        else:
            self.btn_triangle = True

    def on_triangle_release(self):
        self.btn_triangle = False

    def on_circle_press(self):
        if self.is_l2_pressed:
            self.servo1_idx, self.servo2_idx = 2, 2
            print(f"-> Servo: Posisi 3 | S1: {SERVO1_STEPS[2]}°, S2: {SERVO2_STEPS[2]}°")
        else:
            if not self.s1_moving:
                self.s1_moving, self.s1_start_time = True, time.time()

    def on_x_press(self):
        if not self.is_l2_pressed: self.btn_x = True

    def on_x_release(self):
        self.btn_x = False

    # --- CALLBACKS D-PAD (Relay Toggle) ---
    def on_up_arrow_press(self): self.relay1 = 1 - self.relay1
    def on_down_arrow_press(self): self.relay2 = 1 - self.relay2
    def on_left_arrow_press(self): self.relay3 = 1 - self.relay3
    def on_right_arrow_press(self): self.relay4 = 1 - self.relay4

# ==========================================
# THREAD KOMUNIKASI SERIAL
# ==========================================
def serial_loop(ctrl, teensy):
    print("\n--- Background Serial Thread Active ---")
    try:
        while True:
            # 1. Logika Mecanum (Deadzone ~10% dari 32767)
            rx = ctrl.rx if abs(ctrl.rx) > 3200 else 0
            ly = ctrl.ly if abs(ctrl.ly) > 3200 else 0
            lx = ctrl.lx if abs(ctrl.lx) > 3200 else 0

            current_max_speed = BOOST_MECANUM_SPEED if ctrl.is_r2_pressed else DEFAULT_MECANUM_SPEED
            
            # Pada pyPS4Controller, up = negatif. Supaya up jadi positif (maju), kita kalikan -ly
            r_val = map_value(rx, -32767, 32767, -current_max_speed, current_max_speed)
            y_val = map_value(-ly, -32767, 32767, -current_max_speed, current_max_speed)
            x_val = map_value(lx, -32767, 32767, -current_max_speed, current_max_speed)

            # 2. Logika Slider Timer
            current_time = time.time()
            s1_val, s2_val = 0, 0

            if ctrl.s1_moving:
                if (current_time - ctrl.s1_start_time) < SLIDER1_DURATION_SEC:
                    s1_val = MAX_SLIDER1_SPEED if not ctrl.s1_is_out else -MAX_SLIDER1_SPEED
                else:
                    ctrl.s1_moving, ctrl.s1_is_out = False, not ctrl.s1_is_out

            if ctrl.s2_moving:
                if (current_time - ctrl.s2_start_time) < SLIDER2_DURATION_SEC:
                    s2_val = MAX_SLIDER2_SPEED if not ctrl.s2_is_out else -MAX_SLIDER2_SPEED
                else:
                    ctrl.s2_moving, ctrl.s2_is_out = False, not ctrl.s2_is_out

            # 3. Logika Lift
            lift_left, lift_right = 0, 0
            if not ctrl.is_l2_pressed:
                if ctrl.btn_triangle and not ctrl.btn_x:
                    lift_left, lift_right = UP_SPEED_LEFT, UP_SPEED_RIGHT
                elif ctrl.btn_x and not ctrl.btn_triangle:
                    lift_left, lift_right = -DOWN_SPEED_LEFT, -DOWN_SPEED_RIGHT

            # 4. Kirim Data via Serial
            if teensy and teensy.is_open:
                teensy.write(f"<{x_val},{y_val},{r_val}>\n".encode('utf-8'))
                teensy.write(f"S,{s1_val},{s2_val}\n".encode('utf-8'))
                teensy.write(f"J,{lift_left},{lift_right}\n".encode('utf-8'))
                teensy.write(f"R,{ctrl.relay1},{ctrl.relay2},{ctrl.relay3},{ctrl.relay4}\n".encode('utf-8'))
                
                servo1_angle = SERVO1_STEPS[ctrl.servo1_idx]
                servo2_angle = SERVO2_STEPS[ctrl.servo2_idx]
                teensy.write(f"V,{servo1_angle},{servo2_angle}\n".encode('utf-8'))

            time.sleep(0.05) # Loop Rate 20 Hz
    
    except Exception as e:
        print(f"Serial Error/Thread Stopped: {e}")

# ==========================================
# ENTRY POINT
# ==========================================
def main():
    print("==================================================")
    print("MAIN GATEWAY AKTIF: pyPS4Controller")
    print("==================================================")
    
    # 1. Inisiasi Serial
    teensy = None
    try:
        teensy = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        print(f"Teensy Terhubung di {SERIAL_PORT}")
    except Exception as e:
        print(f"Peringatan: Gagal membuka {SERIAL_PORT}. Berjalan dalam mode debug.")

    # 2. Inisiasi Controller Instance
    controller = RobotController(interface=CONTROLLER_INTERFACE, connecting_using_ds4drv=False)

    # 3. Jalankan Background Thread untuk Komunikasi Serial
    serial_worker = threading.Thread(target=serial_loop, args=(controller, teensy), daemon=True)
    serial_worker.start()

    # 4. Dengarkan Input Controller (Proses ini Blocking)
    try:
        print("Menunggu Controller Terkoneksi (Tekan Tombol PS)...")
        controller.listen()
    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh user.")
    finally:
        if teensy and teensy.is_open:
            teensy.write("<0,0,0>\nS,0,0\nJ,0,0\nR,0,0,0,0\n".encode('utf-8'))
            teensy.close()
            print("Koneksi Serial Ditutup. Aman.")

if __name__ == "__main__":
    main()