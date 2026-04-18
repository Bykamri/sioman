import serial
import time
import threading
from pyPS4Controller.controller import Controller

# ==========================================
# KONFIGURASI SERIAL (Raspberry Pi 5 -> Teensy 4.1)
# ==========================================
# Di Pi, biasanya Teensy terbaca sebagai /dev/ttyACM0 atau /dev/ttyUSB0
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

# ==========================================
# KONFIGURASI SLIDER 
# ==========================================
MAX_SLIDER1_SPEED = 150      
SLIDER1_DURATION_MS = 2200   
MAX_SLIDER2_SPEED = 150      
SLIDER2_DURATION_MS = 2200   

# ==========================================
# KONFIGURASI JACK LIFT
# ==========================================
UP_SPEED_LEFT  = 255 
UP_SPEED_RIGHT = 217 
DOWN_SPEED_LEFT  = 250
DOWN_SPEED_RIGHT = 217 

# ==========================================
# KONFIGURASI MECANUM
# ==========================================
MAX_MECANUM_SPEED = 100 


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
# KELAS CONTROLLER EVENT-DRIVEN
# ==========================================
class RobotController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        # Variabel State
        self.axis_maju = 0.0
        self.axis_putar = 0.0
        self.axis_geser = 0.0
        
        self.lift_up = False
        self.lift_down = False
        
        self.trigger_slider1 = False
        self.trigger_slider2 = False

    # --- Analog Kiri Y (Maju / Mundur) ---
    # Nilai asli pyPS4: Atas = negatif (-32767), Bawah = positif (32767)
    # Di-invert agar Atas = +1.0
    def on_L3_up(self, value): self.axis_maju = -value / 32767.0
    def on_L3_down(self, value): self.axis_maju = -value / 32767.0
    def on_L3_y_at_rest(self): self.axis_maju = 0.0

    # --- Analog Kiri X (Putar Kiri / Kanan) ---
    def on_L3_left(self, value): self.axis_putar = value / 32767.0
    def on_L3_right(self, value): self.axis_putar = value / 32767.0
    def on_L3_x_at_rest(self): self.axis_putar = 0.0

    # --- Analog Kanan X (Geser Kiri / Kanan) ---
    def on_R3_left(self, value): self.axis_geser = value / 32767.0
    def on_R3_right(self, value): self.axis_geser = value / 32767.0
    def on_R3_x_at_rest(self): self.axis_geser = 0.0

    # --- Tombol Jack Lift ---
    def on_triangle_press(self): self.lift_up = True
    def on_triangle_release(self): self.lift_up = False
    
    def on_x_press(self): self.lift_down = True
    def on_x_release(self): self.lift_down = False

    # --- Tombol Slider (Trigger) ---
    def on_circle_press(self): self.trigger_slider1 = True
    def on_square_press(self): self.trigger_slider2 = True

# ==========================================
# THREAD PENGIRIMAN SERIAL
# ==========================================
def serial_loop(controller, teensy, stop_event):
    # State Slider
    s1_moving, s1_is_out, s1_start_time = False, False, 0
    s2_moving, s2_is_out, s2_start_time = False, False, 0

    while not stop_event.is_set():
        current_time = time.time() * 1000 

        # 1. LOGIKA MECANUM
        maju  = controller.axis_maju
        putar = controller.axis_putar
        geser = controller.axis_geser

        if abs(maju) < 0.1: maju = 0
        if abs(putar) < 0.1: putar = 0
        if abs(geser) < 0.1: geser = 0

        r_val = map_value(putar, -1.0, 1.0, -MAX_MECANUM_SPEED, MAX_MECANUM_SPEED)
        y_val = map_value(maju, -1.0, 1.0, -MAX_MECANUM_SPEED, MAX_MECANUM_SPEED)
        x_val = map_value(geser, -1.0, 1.0, -MAX_MECANUM_SPEED, MAX_MECANUM_SPEED)

        # 2. LOGIKA SLIDER
        s1_val = 0
        s2_val = 0

        if controller.trigger_slider1 and not s1_moving:
            s1_moving = True
            s1_start_time = current_time
        controller.trigger_slider1 = False 

        if s1_moving:
            if current_time - s1_start_time < SLIDER1_DURATION_MS:
                s1_val = MAX_SLIDER1_SPEED if not s1_is_out else -MAX_SLIDER1_SPEED
            else:
                s1_moving = False
                s1_is_out = not s1_is_out

        if controller.trigger_slider2 and not s2_moving:
            s2_moving = True
            s2_start_time = current_time
        controller.trigger_slider2 = False 

        if s2_moving:
            if current_time - s2_start_time < SLIDER2_DURATION_MS:
                s2_val = MAX_SLIDER2_SPEED if not s2_is_out else -MAX_SLIDER2_SPEED
            else:
                s2_moving = False
                s2_is_out = not s2_is_out

        # 3. LOGIKA JACK LIFT
        lift_left = 0
        lift_right = 0

        if controller.lift_up and not controller.lift_down:
            lift_left = UP_SPEED_LEFT
            lift_right = UP_SPEED_RIGHT
        elif controller.lift_down and not controller.lift_up:
            lift_left = -DOWN_SPEED_LEFT
            lift_right = -DOWN_SPEED_RIGHT

        # 4. TRANSMISI DATA
        cmd_mecanum = f"<{x_val},{y_val},{r_val}>\n"
        teensy.write(cmd_mecanum.encode('utf-8'))

        cmd_slider = f"S,{s1_val},{s2_val}\n"
        teensy.write(cmd_slider.encode('utf-8'))

        cmd_lift = f"J,{lift_left},{lift_right}\n"
        teensy.write(cmd_lift.encode('utf-8'))

        time.sleep(0.05) # Loop ~20Hz

def main():
    print("==================================================")
    print("MENGHUBUNGKAN TEENSY...")
    teensy = init_serial()
    if not teensy:
        return
    print("TEENSY TERHUBUNG!")
    print("==================================================")
    print("MENCARI PS4 CONTROLLER DI /dev/input/js0...")

    # Inisialisasi Kelas Controller
    # Ubah /dev/input/js0 jika controller terbaca di interface lain (misal js1)
    controller = RobotController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    stop_event = threading.Event()

    # Memulai Thread Serial Pengirim (Berjalan di background)
    sender_thread = threading.Thread(target=serial_loop, args=(controller, teensy, stop_event))
    sender_thread.daemon = True
    sender_thread.start()

    print("MAIN GATEWAY AKTIF (Headless Mode)")
    print("Tekan CTRL+C di terminal untuk berhenti.")
    print("==================================================")

    try:
        # Memulai blocking listener dari pyPS4Controller
        controller.listen()
    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh user.")
    finally:
        # Stop background thread
        stop_event.set()
        sender_thread.join(timeout=1.0)
        
        # Kirim perintah stop darurat
        if teensy and teensy.is_open:
            print("Mematikan semua motor...")
            teensy.write("<0,0,0>\n".encode('utf-8'))
            teensy.write("S,0,0\n".encode('utf-8'))
            teensy.write("J,0,0\n".encode('utf-8'))
            time.sleep(0.1)
            teensy.close()

if __name__ == "__main__":
    main()