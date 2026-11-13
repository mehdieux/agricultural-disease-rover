import cv2 as cv
import numpy as np
import time
import socket
import os

# ============================================================
#  FARMLAND ALLEY ROVER - AI PILOT (HEADLESS / PHONE VERSION)
#  Phone-mounted vision system -> ESP32 motor controller (UDP)
#
#  Differences from the laptop version:
#   - No cv.imshow / namedWindow / trackbars / waitKey (no GUI
#     on Android). HSV values are hardcoded below - tune these
#     on a laptop first using a recorded clip or photos from
#     the actual alley, then paste the final numbers in here.
#   - Loop stops on Ctrl+C (or Pydroid's Stop button) instead
#     of a key press.
#   - Optional periodic debug frame saving to disk, since you
#     can't watch a live window - check saved frames afterward
#     to confirm the masks/spine/detections look right.
# ============================================================

# --- 1. NETWORK SETUP ---
ESP32_IP   = "192.168.137.127"   # UPDATE this to match your ESP32's actual IP
UDP_PORT   = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

SEND_INTERVAL = 0.05        # Max command rate: 20 Hz
last_send_time = 0.0
last_cmd_sent  = ""

def send_to_rover(msg):
    """Rate-limited UDP send. Skips duplicate consecutive commands."""
    global last_send_time, last_cmd_sent
    now = time.time()
    if now - last_send_time < SEND_INTERVAL:
        return
    if msg == last_cmd_sent:
        last_send_time = now
        return
    try:
        sock.sendto(msg.encode(), (ESP32_IP, UDP_PORT))
        last_cmd_sent  = msg
        last_send_time = now
        print(f"[CMD] -> {msg}")
    except Exception as e:
        print(f"[SEND ERROR] {e}")

# --- 2. HSV COLOR PROFILES (HARDCODED - tune on laptop first!) ---
# Replace these with the values you land on after a proper tuning
# session. These starting values are the same defaults as the
# laptop script; they will very likely need adjustment for your
# actual lighting/soil/plants.
DIRT_LOW   = np.array([8,  25, 30])
DIRT_HIGH  = np.array([30, 180, 200])
PLANT_LOW  = np.array([38, 60, 50])
PLANT_HIGH = np.array([88, 255, 255])

MORPH_SMALL = np.ones((3, 3), np.uint8)
MORPH_MED   = np.ones((5, 5), np.uint8)

# --- 3. CAMERA SETUP ---
cap = cv.VideoCapture(0)   # If this opens the wrong camera (front vs back),
                            # try index 1.
cap.set(cv.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv.CAP_PROP_FPS, 30)

if not cap.isOpened():
    raise RuntimeError("Could not open camera. Check Pydroid camera permission.")

# --- 4. DEBUG FRAME SAVING (optional, since there's no live view) ---
DEBUG_SAVE       = True          # set False to disable entirely
DEBUG_SAVE_EVERY = 5.0           # seconds between saved debug frames
DEBUG_DIR        = "/sdcard/rover_debug"   # visible in phone's Files app
last_debug_save  = 0.0

if DEBUG_SAVE and not os.path.exists(DEBUG_DIR):
    os.makedirs(DEBUG_DIR, exist_ok=True)

# --- 5. STATE ---
is_slowing        = False
last_plant_time   = 0.0
PLANT_COOLDOWN    = 3.0
SLOW_DURATION     = 2.0
lost_path_count    = 0
LOST_THRESHOLD     = 8

DEADZONE   = 0.18
HARD_TURN  = 0.50

MIN_DIRT_AREA  = 1800
MIN_PLANT_AREA = 2500

print("=" * 50)
print("  FARMLAND ROVER - AI PILOT STARTED (headless)")
print("  Press Ctrl+C or use Pydroid's Stop button to quit")
print("=" * 50)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Camera frame dropped.")
            time.sleep(0.05)
            continue

        try:
            data, addr = sock.recvfrom(1024)
            print(f"[ESP32] {data.decode().strip()}")
        except (BlockingIOError, socket.error):
            pass

        h, w = frame.shape[:2]
        cx_frame  = w // 2
        current_t = time.time()

        # --------------------------------------------------------
        #  6. ROI DEFINITIONS
        # --------------------------------------------------------
        path_top    = int(h * 0.40)
        path_bottom = int(h * 0.95)
        path_left   = int(w * 0.10)
        path_right  = int(w * 0.90)

        plant_top    = int(h * 0.10)
        plant_bottom = int(h * 0.65)
        plant_left   = 0
        plant_right  = w

        path_roi  = frame[path_top:path_bottom,  path_left:path_right]
        plant_roi = frame[plant_top:plant_bottom, plant_left:plant_right]

        # --------------------------------------------------------
        #  7. DIRT MASK
        # --------------------------------------------------------
        hsv_path  = cv.cvtColor(path_roi, cv.COLOR_BGR2HSV)
        mask_dirt = cv.inRange(hsv_path, DIRT_LOW, DIRT_HIGH)
        mask_dirt = cv.morphologyEx(mask_dirt, cv.MORPH_OPEN,  MORPH_SMALL)
        mask_dirt = cv.morphologyEx(mask_dirt, cv.MORPH_CLOSE, MORPH_MED)

        # --------------------------------------------------------
        #  8. STRIP-BASED CENTERLINE
        # --------------------------------------------------------
        STRIP_H   = 20
        roi_h     = path_bottom - path_top
        spine_pts = []

        for y_local in range(roi_h - STRIP_H, -1, -STRIP_H):
            row_mask = mask_dirt[y_local: y_local + STRIP_H, :]
            cols = np.where(row_mask.any(axis=0))[0]
            if len(cols) < 5:
                continue
            col_mid  = int((cols[0] + cols[-1]) / 2)
            col_span = cols[-1] - cols[0]
            if col_span < 10:
                continue
            gx = col_mid + path_left
            gy = y_local + path_top + STRIP_H // 2
            spine_pts.append((gx, gy))

        nav_cmd = "STOP"
        error   = 0.0

        if len(spine_pts) >= 2:
            lost_path_count = 0
            nearest_x, nearest_y = spine_pts[0]
            error = (nearest_x - cx_frame) / (w / 2)

            if   error < -HARD_TURN: nav_cmd = "HARD_LEFT"
            elif error < -DEADZONE:  nav_cmd = "LEFT"
            elif error >  HARD_TURN: nav_cmd = "HARD_RIGHT"
            elif error >  DEADZONE:  nav_cmd = "RIGHT"
            else:                    nav_cmd = "FORWARD"
        else:
            lost_path_count += 1
            if lost_path_count >= LOST_THRESHOLD:
                nav_cmd = "SEARCH"

        # --------------------------------------------------------
        #  9. PLANT DETECTION
        # --------------------------------------------------------
        hsv_plant  = cv.cvtColor(plant_roi, cv.COLOR_BGR2HSV)
        mask_plant = cv.inRange(hsv_plant, PLANT_LOW, PLANT_HIGH)
        mask_plant = cv.morphologyEx(mask_plant, cv.MORPH_OPEN,  MORPH_MED)
        mask_plant = cv.morphologyEx(mask_plant, cv.MORPH_CLOSE, MORPH_MED)

        plant_contours, _ = cv.findContours(
            mask_plant, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
        )

        if is_slowing and (current_t - last_plant_time > SLOW_DURATION):
            is_slowing = False
            print("[INFO] Slow period ended. Resuming navigation.")

        if plant_contours and not is_slowing:
            largest_p = max(plant_contours, key=cv.contourArea)
            if cv.contourArea(largest_p) > MIN_PLANT_AREA:
                if (current_t - last_plant_time) > PLANT_COOLDOWN:
                    is_slowing      = True
                    last_plant_time = current_t
                    print("[ALERT] Plant / crop detected - slowing down.")

        # --------------------------------------------------------
        #  10. FINAL COMMAND DECISION
        # --------------------------------------------------------
        if is_slowing and nav_cmd == "FORWARD":
            final_cmd = "SLOW"
        elif is_slowing and nav_cmd in ("LEFT", "HARD_LEFT", "RIGHT", "HARD_RIGHT"):
            final_cmd = f"SLOW_{nav_cmd}"
        else:
            final_cmd = nav_cmd

        send_to_rover(final_cmd)

        # --------------------------------------------------------
        #  11. OPTIONAL DEBUG FRAME SAVE (replaces cv.imshow)
        # --------------------------------------------------------
        if DEBUG_SAVE and (current_t - last_debug_save > DEBUG_SAVE_EVERY):
            debug_frame = frame.copy()
            cv.rectangle(debug_frame, (path_left, path_top), (path_right, path_bottom), (255, 80, 0), 2)
            cv.rectangle(debug_frame, (plant_left, plant_top), (plant_right, plant_bottom), (0, 200, 0), 1)
            for i in range(len(spine_pts) - 1):
                cv.line(debug_frame, spine_pts[i], spine_pts[i + 1], (0, 255, 0), 2)
            cv.putText(debug_frame, f"CMD: {final_cmd} err={error:+.2f}",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 120), 2)
            fname = os.path.join(DEBUG_DIR, f"frame_{int(current_t)}.jpg")
            cv.imwrite(fname, debug_frame)
            last_debug_save = current_t

except KeyboardInterrupt:
    print("[INFO] Stopped by user.")

finally:
    # --- SHUTDOWN ---
    send_to_rover("STOP")
    print("[INFO] Rover stopped. Shutting down.")
    cap.release()
    sock.close()