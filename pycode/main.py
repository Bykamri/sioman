import serial
import time
import threading
from pyPS4Controller.controller import Controller

# ==========================================
# KONFIGURASI SERIAL (Raspberry Pi -> Teensy 4.1)
# ==========================================
# Ganti ke 'COM6' jika ada workaround di Windows, tapi default pyPS4Controller butuh path Linux
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

# ==========================================
# KONFIGURASI KECEPATAN & DURASI
# ==========================================
MAX_SLIDER1_SPEED = 150      
SLIDER1_DURATION_MS = 2500   
MAX_SLIDER2_SPEED = 255      
SLIDER2_DURATION_MS = 1200   

UP_SPEED_LEFT  = 255 
UP_SPEED_RIGHT = 217 
DOWN_SPEED_LEFT  = 248
DOWN_SPEED_RIGHT = 217 

MAX_MECANUM_SPEED = 100 

# ==========================================
# KONFIGURASI STEP SERVO 
# ==========================================
# Index 0 (Square), Index 1 (Triangle), Index 2 (Circle)
SERVO1_STEPS = [30, 120, 190] # Servo Kecil (Gripper)
SERVO2_STEPS = [85, 85, 110] # Servo Besar

def init_serial():
    try:
        teensy = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        return teensy
    except Exception as e:
        print(f"Error membuka port serial {SERIAL_PORT}: {e}")
        return None

def map_value(value, min_in, max_in, min_out, max_out):
    return int((value - min_in) * (max_out - min_out) / (max_in - min_in) + min_out)

# ==========================================
# KELAS PS4 CONTROLLER EVENT LISTENER
# ==========================================
class RobotController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        # Variable State Analog (-1.0 to 1.0 mapping)
        self.axis_maju = 0.0
        self.axis_geser = 0.0
        self.axis_putar = 0.0
        
        # Variable State Tombol
        self.btn_x = 0
        self.btn_circle = 0
        self.btn_square = 0
        self.btn_triangle = 0
        self.is_l2_pressed = False
        
        # Variable Toggle D-Pad
        self.relay1_toggle = False
        self.relay2_toggle = False
        self.relay3_toggle = False
        self.relay4_toggle = False

    # --- KINEMATIKA (Maju/Mundur & Geser) - ANALOG KIRI ---
    # PyPS4Controller memberi nilai dari -32767 hingga 32767. Kita normalisasi ke rentang -1.0 s/d 1.0
    def on_L3_up(self, value): self.axis_maju = -value / 32767.0
    def on_L3_down(self, value): self.axis_maju = -value / 32767.0
    def on_L3_y_at_rest(self): self.axis_maju = 0.0
    
    def on_L3_left(self, value): self.axis_geser = value / 32767.0
    def on_L3_right(self, value): self.axis_geser = value / 32767.0
    def on_L3_x_at_rest(self): self.axis_geser = 0.0
    
    # --- ROTASI - ANALOG KANAN ---
    def on_R3_left(self, value): self.axis_putar = value / 32767.0
    def on_R3_right(self, value): self.axis_putar = value / 32767.0
    def on_R3_x_at_rest(self): self.axis_putar = 0.0

    # --- MODIFIER L2 ---
    def on_L2_press(self, value): self.is_l2_pressed = True
    def on_L2_release(self): self.is_l2_pressed = False

    # --- TOMBOL AKSI ---
    def on_x_press(self): self.btn_x = 1
    def on_x_release(self): self.btn_x = 0
    
    def on_triangle_press(self): self.btn_triangle = 1
    def on_triangle_release(self): self.btn_triangle = 0
    
    def on_circle_press(self): self.btn_circle = 1
    def on_circle_release(self): self.btn_circle = 0
    
    def on_square_press(self): self.btn_square = 1
    def on_square_release(self): self.btn_square = 0

    # --- D-PAD (RELAY) ---
    def on_up_arrow_press(self): self.relay1_toggle = True
    def on_down_arrow_press(self): self.relay2_toggle = True
    def on_left_arrow_press(self): self.relay3_toggle = True
    def on_right_arrow_press(self): self.relay4_toggle = True


# ==========================================
# THREAD LOGIKA UTAMA (Berjalan bersamaan dengan Listener PS4)
# ==========================================
def main_logic_loop(ps4, teensy, stop_event):
    # Variabel State Slider
    s1_moving, s1_is_out, s1_start_time, last_circle_btn = False, False, 0, 0
    s2_moving, s2_is_out, s2_start_time, last_square_btn = False, False, 0, 0
    last_triangle_btn = 0 

    # Variabel State Relay
    relay1_state, relay2_state, relay3_state, relay4_state = 0, 0, 0, 0

    # Variabel State Step Servo (Default di tengah/Index 1)
    servo1_idx = 1  
    servo2_idx = 1  

    while not stop_event.is_set():
        current_time = time.time() * 1000 # Milliseconds

        # Membaca State dari Objek Controller pyPS4Controller
        axis_maju = ps4.axis_maju
        axis_geser = ps4.axis_geser
        axis_putar = ps4.axis_putar
        
        btn_x = ps4.btn_x
        btn_circle = ps4.btn_circle
        btn_square = ps4.btn_square
        btn_triangle = ps4.btn_triangle
        is_l2_pressed = ps4.is_l2_pressed

        # --- 1. LOGIKA MECANUM ---
        if abs(axis_putar) < 0.1: axis_putar = 0
        if abs(axis_maju)  < 0.1: axis_maju = 0
        if abs(axis_geser) < 0.1: axis_geser = 0
        
        r_val = map_value(axis_putar, -1.0, 1.0, -MAX_MECANUM_SPEED, MAX_MECANUM_SPEED)
        y_val = map_value(axis_maju, -1.0, 1.0, -MAX_MECANUM_SPEED, MAX_MECANUM_SPEED)
        x_val = map_value(axis_geser, -1.0, 1.0, -MAX_MECANUM_SPEED, MAX_MECANUM_SPEED)

        # --- 2. LOGIKA KOMBINASI L2 (SERVO) ---
        if is_l2_pressed:
            if btn_square == 1 and last_square_btn == 0:
                servo1_idx = 0; servo2_idx = 0
                print(f"-> Servo: Posisi 1 (Square) | S1: {SERVO1_STEPS[0]}°, S2: {SERVO2_STEPS[0]}°")
            
            elif btn_triangle == 1 and last_triangle_btn == 0:
                servo1_idx = 1; servo2_idx = 1
                print(f"-> Servo: Posisi 2 (Triangle) | S1: {SERVO1_STEPS[1]}°, S2: {SERVO2_STEPS[1]}°")
            
            elif btn_circle == 1 and last_circle_btn == 0:
                servo1_idx = 2; servo2_idx = 2
                print(f"-> Servo: Posisi 3 (Circle) | S1: {SERVO1_STEPS[2]}°, S2: {SERVO2_STEPS[2]}°")

        # --- 3. LOGIKA SLIDER (HANYA AKTIF JIKA L2 TIDAK DITEKAN) ---
        s1_val, s2_val = 0, 0
        
        # Slider Bawah (Circle)
        if btn_circle == 1 and not is_l2_pressed and last_circle_btn == 0 and not s1_moving:
            s1_moving, s1_start_time = True, current_time 
        
        if s1_moving:
            if current_time - s1_start_time < SLIDER1_DURATION_MS:
                s1_val = MAX_SLIDER1_SPEED if not s1_is_out else -MAX_SLIDER1_SPEED
            else:
                s1_moving, s1_is_out = False, not s1_is_out

        # Slider Atas (Square)
        if btn_square == 1 and not is_l2_pressed and last_square_btn == 0 and not s2_moving:
            s2_moving, s2_start_time = True, current_time 

        if s2_moving:
            if current_time - s2_start_time < SLIDER2_DURATION_MS:
                s2_val = MAX_SLIDER2_SPEED if not s2_is_out else -MAX_SLIDER2_SPEED
            else:
                s2_moving, s2_is_out = False, not s2_is_out

        # --- 4. LOGIKA JACK LIFT (HANYA AKTIF JIKA L2 TIDAK DITEKAN) ---
        lift_left, lift_right = 0, 0
        if not is_l2_pressed:
            if btn_triangle and not btn_x:
                lift_left, lift_right = UP_SPEED_LEFT, UP_SPEED_RIGHT
            elif btn_x and not btn_triangle:
                lift_left, lift_right = -DOWN_SPEED_LEFT, -DOWN_SPEED_RIGHT

        # --- 5. LOGIKA RELAY ---
        if ps4.relay1_toggle: relay1_state = 1 - relay1_state; ps4.relay1_toggle = False
        if ps4.relay2_toggle: relay2_state = 1 - relay2_state; ps4.relay2_toggle = False
        if ps4.relay3_toggle: relay3_state = 1 - relay3_state; ps4.relay3_toggle = False
        if ps4.relay4_toggle: relay4_state = 1 - relay4_state; ps4.relay4_toggle = False

        # --- SIMPAN STATE TOMBOL ---
        last_square_btn = btn_square
        last_triangle_btn = btn_triangle
        last_circle_btn = btn_circle

        # --- PENGIRIMAN SERIAL KE TEENSY ---
        try:
            teensy.write(f"<{x_val},{y_val},{r_val}>\n".encode('utf-8'))
            teensy.write(f"S,{s1_val},{s2_val}\n".encode('utf-8'))
            teensy.write(f"J,{lift_left},{lift_right}\n".encode('utf-8'))
            teensy.write(f"R,{relay1_state},{relay2_state},{relay3_state},{relay4_state}\n".encode('utf-8'))
            
            servo1_angle = SERVO1_STEPS[servo1_idx]
            servo2_angle = SERVO2_STEPS[servo2_idx]
            teensy.write(f"V,{servo1_angle},{servo2_angle}\n".encode('utf-8'))
        except Exception as e:
            pass # Cegah crash jika port terputus mendadak

        time.sleep(0.05) # Delay 50ms untuk kestabilan port Serial


def main():
    teensy = init_serial()
    if not teensy:
        return

    print("==================================================")
    print("MAIN GATEWAY AKTIF: MECANUM, SLIDER, LIFT, RELAY & SERVO")
    print("==================================================")
    print("[Analog Kiri]   -> Kinematika XY (Maju/Geser)")
    print("[Analog Kanan X]-> Rotasi (Putar Kiri/Kanan)")
    print("[Lingkaran]     -> Toggle Slider Bawah")
    print("[Persegi]       -> Toggle Slider Atas")
    print("[Segitiga / X]  -> Jack Lift Naik / Turun")
    print("[D-Pad]         -> Toggle Relay 1-4")
    print("--- KOMBINASI SERVO (TAHAN L2) ---")
    print("[L2 + Persegi]  -> Servo ke Step 1 (Index 0)")
    print("[L2 + Segitiga] -> Servo ke Step 2 (Index 1)")
    print("[L2 + Lingkaran]-> Servo ke Step 3 (Index 2)")
    print("==================================================")

    # Inisialisasi Controller
    ps4 = RobotController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    
    # Memulai Thread agar perhitungan logika dan serial.write tidak di-block oleh ps4.listen()
    stop_event = threading.Event()
    logic_thread = threading.Thread(target=main_logic_loop, args=(ps4, teensy, stop_event))
    logic_thread.daemon = True
    logic_thread.start()

    try:
        # listen() bersifat memblokir (blocking call) sampai ada event
        ps4.listen() 
    except KeyboardInterrupt:
        print("\nProgram dihentikan.")
    finally:
        # Menutup program dan mereset Teensy ke 0
        stop_event.set()
        if teensy and teensy.is_open:
            try:
                teensy.write("<0,0,0>\n".encode('utf-8'))
                teensy.write("S,0,0\n".encode('utf-8'))
                teensy.write("J,0,0\n".encode('utf-8'))
                teensy.write("R,0,0,0,0\n".encode('utf-8'))
                teensy.close()
            except:
                pass

if __name__ == "__main__":
    main()