import cv2 as cv
import numpy as np
import time
import socket

# ============================================================
#  FARMLAND ALLEY ROVER - AI PILOT  (Fixed)
#  Fixes applied:
#    1. Camera failure recovery with auto-reconnect
#    2. (reverted — all commands already defined in ESP32 firmware)
#    3. Plant bounding box drawn whenever plant is active
#    4. Weighted spine average for smoother steering
#    5. Camera resolution auto-corrected to what camera actually delivers
#    6. MIN_DIRT_AREA now actually used to filter dirt contours
#    7. Reliable STOP delivery (3x retries on shutdown)
# ============================================================

# --- 1. NETWORK SETUP ---
ESP32_IP   = "192.168.137.127"
UDP_PORT   = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

SEND_INTERVAL = 0.05
last_send_time = 0.0
last_cmd_sent  = ""

def send_to_rover(msg, force=False):
    """
    Rate-limited UDP send. Skips duplicate consecutive commands.
    Set force=True to bypass rate limit (used for critical STOP).
    """
    global last_send_time, last_cmd_sent
    now = time.time()
    if not force and (now - last_send_time < SEND_INTERVAL):
        return
    if not force and msg == last_cmd_sent:
        last_send_time = now
        return
    try:
        sock.sendto(msg.encode(), (ESP32_IP, UDP_PORT))
        last_cmd_sent  = msg
        last_send_time = now
        print(f"[CMD] → {msg}")
    except Exception as e:
        print(f"[SEND ERROR] {e}")

# FIX 7: Reliable STOP — retries several times with a small delay
def send_stop_reliable(retries=5, delay=0.05):
    print("[INFO] Sending reliable STOP...")
    for _ in range(retries):
        try:
            sock.sendto(b"STOP", (ESP32_IP, UDP_PORT))
        except Exception as e:
            print(f"[SEND ERROR] {e}")
        time.sleep(delay)

# --- 2. HSV COLOR PROFILES ---
DIRT_LOW   = np.array([8,  25,  30])
DIRT_HIGH  = np.array([30, 180, 200])
PLANT_LOW  = np.array([38, 60,  50])
PLANT_HIGH = np.array([88, 255, 255])

MORPH_SMALL = np.ones((3, 3), np.uint8)
MORPH_MED   = np.ones((5, 5), np.uint8)

# --- 3. CAMERA SETUP ---
TARGET_W, TARGET_H, TARGET_FPS = 640, 480, 30

# FIX 1: Camera open logic extracted into a function for reconnection
def open_camera(index=0):
    c = cv.VideoCapture(index)
    c.set(cv.CAP_PROP_FRAME_WIDTH,  TARGET_W)
    c.set(cv.CAP_PROP_FRAME_HEIGHT, TARGET_H)
    c.set(cv.CAP_PROP_FPS, TARGET_FPS)
    return c

cap = open_camera()
CAMERA_INDEX      = 0
MAX_DROP_FRAMES   = 15          # Consecutive failures before reconnect attempt
drop_frame_count  = 0

# FIX 5: Read what the camera actually delivers and adopt it as the working resolution.
# Some cameras silently ignore CAP_PROP_FRAME_WIDTH/HEIGHT requests.
actual_w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
if actual_w != TARGET_W or actual_h != TARGET_H:
    print(f"[WARN] Camera gave {actual_w}x{actual_h} instead of {TARGET_W}x{TARGET_H}.")
    print(f"[INFO] Adopting {actual_w}x{actual_h} as working resolution.")
    TARGET_W, TARGET_H = actual_w, actual_h
else:
    print(f"[INFO] Camera resolution confirmed: {TARGET_W}x{TARGET_H}")

# --- 4. TUNING PANEL ---
TUNING_WIN = "HSV Tuning"
cv.namedWindow(TUNING_WIN, cv.WINDOW_NORMAL)
cv.resizeWindow(TUNING_WIN, 400, 300)

cv.createTrackbar("Dirt H Low",  TUNING_WIN, DIRT_LOW[0],  179, lambda x: None)
cv.createTrackbar("Dirt H High", TUNING_WIN, DIRT_HIGH[0], 179, lambda x: None)
cv.createTrackbar("Dirt S Low",  TUNING_WIN, DIRT_LOW[1],  255, lambda x: None)
cv.createTrackbar("Dirt V Low",  TUNING_WIN, DIRT_LOW[2],  255, lambda x: None)
cv.createTrackbar("Plant H Low",  TUNING_WIN, PLANT_LOW[0],  179, lambda x: None)
cv.createTrackbar("Plant H High", TUNING_WIN, PLANT_HIGH[0], 179, lambda x: None)
cv.createTrackbar("Plant S Low",  TUNING_WIN, PLANT_LOW[1],  255, lambda x: None)

def read_trackbars():
    dhl = cv.getTrackbarPos("Dirt H Low",   TUNING_WIN)
    dhh = cv.getTrackbarPos("Dirt H High",  TUNING_WIN)
    dsl = cv.getTrackbarPos("Dirt S Low",   TUNING_WIN)
    dvl = cv.getTrackbarPos("Dirt V Low",   TUNING_WIN)
    phl = cv.getTrackbarPos("Plant H Low",  TUNING_WIN)
    phh = cv.getTrackbarPos("Plant H High", TUNING_WIN)
    psl = cv.getTrackbarPos("Plant S Low",  TUNING_WIN)
    return (
        np.array([dhl, dsl, dvl]), np.array([dhh, 200, 200]),
        np.array([phl, psl, 50]),  np.array([phh, 255, 255])
    )

# --- 5. STATE ---
is_slowing       = False
last_plant_time  = 0.0
PLANT_COOLDOWN   = 3.0
SLOW_DURATION    = 2.0
lost_path_count  = 0
LOST_THRESHOLD   = 8

# FIX 3: Track the last known plant rect so we can keep drawing it
last_plant_rect  = None      # (x, y, w, h) in global frame coords

DEADZONE       = 0.18
HARD_TURN      = 0.50
MIN_DIRT_AREA  = 1800        # FIX 6: will now actually be used
MIN_PLANT_AREA = 2500

print("=" * 50)
print("  FARMLAND ROVER - AI PILOT (FIXED)")
print("  [q] Quit   [r] Reset state   [s] Save frame")
print("=" * 50)

while True:
    ret, frame = cap.read()

    # --------------------------------------------------------
    # FIX 1: Camera failure recovery with auto-reconnect
    # --------------------------------------------------------
    if not ret:
        drop_frame_count += 1
        print(f"[WARN] Frame drop #{drop_frame_count}")
        if drop_frame_count >= MAX_DROP_FRAMES:
            print("[ERROR] Too many dropped frames. Attempting camera reconnect...")
            cap.release()
            time.sleep(1.0)
            cap = open_camera(CAMERA_INDEX)
            drop_frame_count = 0
            actual_w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
            print(f"[INFO] Reconnected: {actual_w}x{actual_h}")
        else:
            time.sleep(0.05)
        continue

    drop_frame_count = 0   # Reset on successful frame

    # FIX 5: Read actual frame dimensions every loop (handles resize changes)
    h, w = frame.shape[:2]
    cx_frame  = w // 2
    current_t = time.time()

    try:
        data, _ = sock.recvfrom(1024)
        print(f"[ESP32] {data.decode().strip()}")
    except (BlockingIOError, socket.error):
        pass

    dirt_low, dirt_high, plant_low, plant_high = read_trackbars()

    # --------------------------------------------------------
    # 6. ROI DEFINITIONS  (always derived from live h, w)
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
    # 7. DIRT MASK
    # --------------------------------------------------------
    hsv_path  = cv.cvtColor(path_roi, cv.COLOR_BGR2HSV)
    mask_dirt = cv.inRange(hsv_path, dirt_low, dirt_high)
    mask_dirt = cv.morphologyEx(mask_dirt, cv.MORPH_OPEN,  MORPH_SMALL)
    mask_dirt = cv.morphologyEx(mask_dirt, cv.MORPH_CLOSE, MORPH_MED)

    # --------------------------------------------------------
    # FIX 6: Filter dirt by contour area before using mask
    # Removes dust / pebble noise that passed morphology.
    # --------------------------------------------------------
    dirt_contours, _ = cv.findContours(mask_dirt, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    filtered_mask = np.zeros_like(mask_dirt)
    for cnt in dirt_contours:
        if cv.contourArea(cnt) >= MIN_DIRT_AREA:
            cv.drawContours(filtered_mask, [cnt], -1, 255, -1)
    mask_dirt = filtered_mask

    # --------------------------------------------------------
    # 8. STRIP-BASED CENTERLINE
    # FIX 4: Weighted average of ALL spine points (weight = proximity
    #         to rover, i.e. lower strips count more) for smoother steering.
    # --------------------------------------------------------
    STRIP_H   = 20
    roi_h     = path_bottom - path_top
    spine_pts = []

    for y_local in range(roi_h - STRIP_H, -1, -STRIP_H):
        row_mask = mask_dirt[y_local : y_local + STRIP_H, :]
        cols     = np.where(row_mask.any(axis=0))[0]
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

        # FIX 4: Weighted average — strips closer to rover (index 0) get higher weight
        n       = len(spine_pts)
        weights = np.array([n - i for i in range(n)], dtype=float)
        weights /= weights.sum()
        avg_x   = int(sum(spine_pts[i][0] * weights[i] for i in range(n)))

        # Still use nearest point for the red dot (most immediate steering cue)
        nearest_x, nearest_y = spine_pts[0]

        # Steering error from weighted average (smoother on curves)
        error = (avg_x - cx_frame) / (w / 2)

        if   error < -HARD_TURN:  nav_cmd = "HARD_LEFT"
        elif error < -DEADZONE:   nav_cmd = "LEFT"
        elif error >  HARD_TURN:  nav_cmd = "HARD_RIGHT"
        elif error >  DEADZONE:   nav_cmd = "RIGHT"
        else:                     nav_cmd = "FORWARD"

        for i in range(n - 1):
            cv.line(frame, spine_pts[i], spine_pts[i+1], (0, 255, 0), 2)
        cv.circle(frame, (nearest_x, nearest_y), 10, (0, 0, 255), -1)
        # Show the weighted average point in yellow
        cv.circle(frame, (avg_x, nearest_y), 7, (0, 255, 255), -1)

    else:
        lost_path_count += 1
        if lost_path_count >= LOST_THRESHOLD:
            nav_cmd = "SEARCH"

    # --------------------------------------------------------
    # 9. PLANT DETECTION
    # --------------------------------------------------------
    hsv_plant  = cv.cvtColor(plant_roi, cv.COLOR_BGR2HSV)
    mask_plant = cv.inRange(hsv_plant, plant_low, plant_high)
    mask_plant = cv.morphologyEx(mask_plant, cv.MORPH_OPEN,  MORPH_MED)
    mask_plant = cv.morphologyEx(mask_plant, cv.MORPH_CLOSE, MORPH_MED)

    plant_contours, _ = cv.findContours(
        mask_plant, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
    )

    if is_slowing and (current_t - last_plant_time > SLOW_DURATION):
        is_slowing      = False
        last_plant_rect = None      # FIX 3: clear stale box when slow ends
        print("[INFO] Slow period ended. Resuming navigation.")

    if plant_contours:
        largest_p = max(plant_contours, key=cv.contourArea)
        if cv.contourArea(largest_p) > MIN_PLANT_AREA:
            x, y, pw, ph = cv.boundingRect(largest_p)
            # FIX 3: Always update the stored global rect while plant is visible
            last_plant_rect = (
                x + plant_left, y + plant_top,
                x + plant_left + pw, y + plant_top + ph
            )
            if not is_slowing and (current_t - last_plant_time) > PLANT_COOLDOWN:
                is_slowing      = True
                last_plant_time = current_t
                print("[ALERT] Plant / crop detected — slowing down.")
    else:
        # Plant no longer visible but slow may still be active — keep last rect
        if not is_slowing:
            last_plant_rect = None

    # FIX 3: Draw bounding box whenever we have a recent plant position
    if last_plant_rect is not None:
        x1, y1, x2, y2 = last_plant_rect
        cv.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)

    # --------------------------------------------------------
    # 10. FINAL COMMAND DECISION
    # FIX 2: Composite commands mapped to defined ESP32 tokens only
    # --------------------------------------------------------
    if is_slowing:
        if nav_cmd == "FORWARD":
            final_cmd = "SLOW"
        elif nav_cmd in ("LEFT", "HARD_LEFT"):
            final_cmd = "SLOW_TURN_LEFT"    # Defined in VALID_COMMANDS
        elif nav_cmd in ("RIGHT", "HARD_RIGHT"):
            final_cmd = "SLOW_TURN_RIGHT"   # Defined in VALID_COMMANDS
        else:
            final_cmd = nav_cmd             # STOP / SEARCH pass through unchanged
    else:
        final_cmd = nav_cmd

    send_to_rover(final_cmd)

    # --------------------------------------------------------
    # 11. OVERLAY / DEBUG VISUALS
    # --------------------------------------------------------
    cv.rectangle(frame,
                 (path_left, path_top), (path_right, path_bottom),
                 (255, 80, 0), 2)
    cv.rectangle(frame,
                 (plant_left, plant_top), (plant_right, plant_bottom),
                 (0, 200, 0), 1)
    cv.line(frame, (cx_frame, path_top), (cx_frame, path_bottom),
            (200, 200, 200), 1)

    bar_color = (0, 180, 255) if is_slowing else (0, 255, 120)
    cv.rectangle(frame, (0, 0), (w, 55), (20, 20, 20), -1)
    cv.putText(frame, f"CMD: {final_cmd}",
               (10, 38), cv.FONT_HERSHEY_SIMPLEX, 1.1, bar_color, 2)
    cv.putText(frame, f"err={error:+.2f}",
               (w - 170, 38), cv.FONT_HERSHEY_SIMPLEX, 0.9, (180, 180, 180), 2)

    if lost_path_count >= LOST_THRESHOLD:
        cv.putText(frame, "!! PATH LOST !!",
                   (cx_frame - 120, h // 2),
                   cv.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

    cv.imshow("Rover Vision", frame)

    mask_display = cv.cvtColor(mask_dirt, cv.COLOR_GRAY2BGR)
    for i in range(len(spine_pts) - 1):
        px, py = spine_pts[i]
        qx, qy = spine_pts[i+1]
        cv.line(mask_display,
                (px - path_left, py - path_top),
                (qx - path_left, qy - path_top),
                (0, 255, 0), 2)
    cv.imshow("Dirt Mask", mask_display)

    # --------------------------------------------------------
    # 12. KEYBOARD CONTROLS
    # --------------------------------------------------------
    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('r'):
        is_slowing      = False
        lost_path_count = 0
        last_cmd_sent   = ""
        last_plant_rect = None
        print("[INFO] State reset.")
    elif key == ord('s'):
        fname = f"frame_{int(current_t)}.jpg"
        cv.imwrite(fname, frame)
        print(f"[INFO] Saved {fname}")

# --- SHUTDOWN ---
# FIX 7: Reliable multi-retry STOP before closing
send_stop_reliable(retries=5, delay=0.05)
print("[INFO] Rover stopped. Shutting down.")
cap.release()
cv.destroyAllWindows()
sock.close()
