import cv2 as cv
import numpy as np
import time
import socket

# ============================================================
#  FARMLAND ALLEY ROVER - AI PILOT
#  Phone-mounted vision system → ESP32 motor controller (UDP)
# ============================================================

# --- 1. NETWORK SETUP ---
ESP32_IP   = "192.168.137.127"
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
    if msg == last_cmd_sent:          # Suppress identical repeat
        last_send_time = now
        return
    try:
        sock.sendto(msg.encode(), (ESP32_IP, UDP_PORT))
        last_cmd_sent  = msg
        last_send_time = now
        print(f"[CMD] → {msg}")
    except Exception as e:
        print(f"[SEND ERROR] {e}")

# --- 2. HSV COLOR PROFILES ---
# Farmland alleys: bare soil / dry earth between crop rows.
# These assume natural outdoor sunlight. Adjust via trackbars if needed.

# Dirt / bare soil: warm browns, tans, dry clay
DIRT_LOW  = np.array([8,  25, 30])
DIRT_HIGH = np.array([30, 180, 200])

# Crop / plant green: vibrant field greens, NOT yellow-green weeds
PLANT_LOW  = np.array([38, 60, 50])
PLANT_HIGH = np.array([88, 255, 255])

# Morphology kernels
MORPH_SMALL = np.ones((3, 3), np.uint8)
MORPH_MED   = np.ones((5, 5), np.uint8)

# --- 3. CAMERA SETUP ---
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv.CAP_PROP_FPS, 30)

# --- 4. TUNING PANEL (Trackbars) ---
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
    dhl  = cv.getTrackbarPos("Dirt H Low",   TUNING_WIN)
    dhh  = cv.getTrackbarPos("Dirt H High",  TUNING_WIN)
    dsl  = cv.getTrackbarPos("Dirt S Low",   TUNING_WIN)
    dvl  = cv.getTrackbarPos("Dirt V Low",   TUNING_WIN)
    phl  = cv.getTrackbarPos("Plant H Low",  TUNING_WIN)
    phh  = cv.getTrackbarPos("Plant H High", TUNING_WIN)
    psl  = cv.getTrackbarPos("Plant S Low",  TUNING_WIN)
    dirt_low  = np.array([dhl, dsl, dvl])
    dirt_high = np.array([dhh, 200, 200])
    plant_low  = np.array([phl, psl, 50])
    plant_high = np.array([phh, 255, 255])
    return dirt_low, dirt_high, plant_low, plant_high

# --- 5. STATE ---
is_slowing        = False
last_plant_time   = 0.0
PLANT_COOLDOWN    = 3.0
SLOW_DURATION     = 2.0
lost_path_count   = 0
LOST_THRESHOLD    = 8          # Frames before declaring path lost

# Dead-zone / steering thresholds (fraction of half-width)
DEADZONE   = 0.18
HARD_TURN  = 0.50              # Beyond this → sharp turn command

# Minimum dirt contour area (px²) — filters out dust/noise
MIN_DIRT_AREA  = 1800
MIN_PLANT_AREA = 2500

print("=" * 50)
print("  FARMLAND ROVER - AI PILOT STARTED")
print("  [q] Quit   [r] Reset state   [s] Save frame")
print("=" * 50)

while True:
    ret, frame = cap.read()
    if not ret:
        print("[WARN] Camera frame dropped.")
        time.sleep(0.05)
        continue

    # --- Receive ESP32 feedback ---
    try:
        data, addr = sock.recvfrom(1024)
        print(f"[ESP32] {data.decode().strip()}")
    except (BlockingIOError, socket.error):
        pass

    h, w = frame.shape[:2]
    cx_frame   = w // 2
    current_t  = time.time()

    # Read live HSV ranges from trackbars
    dirt_low, dirt_high, plant_low, plant_high = read_trackbars()

    # --------------------------------------------------------
    #  6. ROI DEFINITIONS
    #  Path ROI : lower 55% of frame, nearly full width
    #             (alley dirt is directly below the rover)
    #  Plant ROI: upper-mid 60% of frame, full width
    #             (crops are on both sides, not just right)
    # --------------------------------------------------------
    path_top    = int(h * 0.40)
    path_bottom = int(h * 0.95)
    path_left   = int(w * 0.10)
    path_right  = int(w * 0.90)

    plant_top    = int(h * 0.10)
    plant_bottom = int(h * 0.65)
    # Full width for plants — crops appear on either side of the alley
    plant_left   = 0
    plant_right  = w

    path_roi  = frame[path_top:path_bottom,  path_left:path_right]
    plant_roi = frame[plant_top:plant_bottom, plant_left:plant_right]

    # --------------------------------------------------------
    #  7. DIRT MASK
    # --------------------------------------------------------
    hsv_path  = cv.cvtColor(path_roi, cv.COLOR_BGR2HSV)
    mask_dirt = cv.inRange(hsv_path, dirt_low, dirt_high)
    # Clean up speckling from pebbles / shadows
    mask_dirt = cv.morphologyEx(mask_dirt, cv.MORPH_OPEN,  MORPH_SMALL)
    mask_dirt = cv.morphologyEx(mask_dirt, cv.MORPH_CLOSE, MORPH_MED)

    # --------------------------------------------------------
    #  8. STRIP-BASED CENTERLINE  (true spine of the dirt path)
    #  Slice mask into horizontal bands → find midpoint of dirt
    #  in each band → draw the spine and compute steering error
    #  from the bottom-most reliable point (closest to rover).
    # --------------------------------------------------------
    STRIP_H    = 20            # px per strip
    roi_h      = path_bottom - path_top
    spine_pts  = []            # (global_x, global_y)

    for y_local in range(roi_h - STRIP_H, -1, -STRIP_H):
        row_mask = mask_dirt[y_local : y_local + STRIP_H, :]
        cols     = np.where(row_mask.any(axis=0))[0]
        if len(cols) < 5:       # Not enough dirt pixels in this strip
            continue
        # Remove outlier columns (shadow edges, stray pixels)
        col_mid  = int((cols[0] + cols[-1]) / 2)
        col_span = cols[-1] - cols[0]
        if col_span < 10:       # Sliver — skip
            continue
        gx = col_mid  + path_left
        gy = y_local  + path_top + STRIP_H // 2
        spine_pts.append((gx, gy))

    nav_cmd = "STOP"
    error   = 0.0

    if len(spine_pts) >= 2:
        lost_path_count = 0

        # Use the lowest (nearest) spine point for steering
        nearest_x, nearest_y = spine_pts[0]
        error = (nearest_x - cx_frame) / (w / 2)   # −1 (far left) … +1 (far right)

        if   error < -HARD_TURN:  nav_cmd = "HARD_LEFT"
        elif error < -DEADZONE:   nav_cmd = "LEFT"
        elif error >  HARD_TURN:  nav_cmd = "HARD_RIGHT"
        elif error >  DEADZONE:   nav_cmd = "RIGHT"
        else:                     nav_cmd = "FORWARD"

        # Draw spine
        for i in range(len(spine_pts) - 1):
            cv.line(frame, spine_pts[i], spine_pts[i+1], (0, 255, 0), 2)
        cv.circle(frame, (nearest_x, nearest_y), 10, (0, 0, 255), -1)

    else:
        lost_path_count += 1
        if lost_path_count >= LOST_THRESHOLD:
            nav_cmd = "SEARCH"   # ESP32 should do a slow spin to re-acquire

    # --------------------------------------------------------
    #  9. PLANT DETECTION  (full-width, both sides of alley)
    # --------------------------------------------------------
    hsv_plant  = cv.cvtColor(plant_roi, cv.COLOR_BGR2HSV)
    mask_plant = cv.inRange(hsv_plant, plant_low, plant_high)
    mask_plant = cv.morphologyEx(mask_plant, cv.MORPH_OPEN,  MORPH_MED)
    mask_plant = cv.morphologyEx(mask_plant, cv.MORPH_CLOSE, MORPH_MED)

    plant_contours, _ = cv.findContours(
        mask_plant, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
    )

    # End slow period if it's expired
    if is_slowing and (current_t - last_plant_time > SLOW_DURATION):
        is_slowing = False
        print("[INFO] Slow period ended. Resuming navigation.")

    # Trigger new slow on significant plant detection
    if plant_contours and not is_slowing:
        largest_p = max(plant_contours, key=cv.contourArea)
        if cv.contourArea(largest_p) > MIN_PLANT_AREA:
            if (current_t - last_plant_time) > PLANT_COOLDOWN:
                is_slowing       = True
                last_plant_time  = current_t
                print("[ALERT] Plant / crop detected — slowing down.")
                # Draw bounding box on the detected plant
                x, y, pw, ph = cv.boundingRect(largest_p)
                cv.rectangle(
                    frame,
                    (x + plant_left, y + plant_top),
                    (x + plant_left + pw, y + plant_top + ph),
                    (0, 165, 255), 2
                )

    # --------------------------------------------------------
    #  10. FINAL COMMAND DECISION
    #  SLOW overrides nav but does NOT override STOP / SEARCH
    # --------------------------------------------------------
    if is_slowing and nav_cmd == "FORWARD":
        final_cmd = "SLOW"
    elif is_slowing and nav_cmd in ("LEFT", "HARD_LEFT", "RIGHT", "HARD_RIGHT"):
        final_cmd = f"SLOW_{nav_cmd}"    # e.g. "SLOW_LEFT" — ESP32 turns slowly
    else:
        final_cmd = nav_cmd

    send_to_rover(final_cmd)

    # --------------------------------------------------------
    #  11. OVERLAY / DEBUG VISUALS
    # --------------------------------------------------------
    # Path ROI box
    cv.rectangle(frame,
                 (path_left, path_top), (path_right, path_bottom),
                 (255, 80, 0), 2)
    # Plant ROI box
    cv.rectangle(frame,
                 (plant_left, plant_top), (plant_right, plant_bottom),
                 (0, 200, 0), 1)
    # Center reference line
    cv.line(frame, (cx_frame, path_top), (cx_frame, path_bottom),
            (200, 200, 200), 1)

    # Status bar
    bar_color = (0, 180, 255) if is_slowing else (0, 255, 120)
    cv.rectangle(frame, (0, 0), (w, 55), (20, 20, 20), -1)
    cv.putText(frame, f"CMD: {final_cmd}",
               (10, 38), cv.FONT_HERSHEY_SIMPLEX, 1.1, bar_color, 2)
    err_str = f"err={error:+.2f}"
    cv.putText(frame, err_str,
               (w - 170, 38), cv.FONT_HERSHEY_SIMPLEX, 0.9, (180, 180, 180), 2)

    # Lost path warning
    if lost_path_count >= LOST_THRESHOLD:
        cv.putText(frame, "!! PATH LOST !!",
                   (cx_frame - 120, h // 2),
                   cv.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

    cv.imshow("Rover Vision", frame)

    # Show dirt mask for debugging
    mask_display = cv.cvtColor(mask_dirt, cv.COLOR_GRAY2BGR)
    # Draw spine on mask too
    for i in range(len(spine_pts) - 1):
        px, py = spine_pts[i]
        qx, qy = spine_pts[i+1]
        cv.line(mask_display,
                (px - path_left, py - path_top),
                (qx - path_left, qy - path_top),
                (0, 255, 0), 2)
    cv.imshow("Dirt Mask", mask_display)

    # --------------------------------------------------------
    #  12. KEYBOARD CONTROLS
    # --------------------------------------------------------
    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('r'):
        is_slowing      = False
        lost_path_count = 0
        last_cmd_sent   = ""
        print("[INFO] State reset.")
    elif key == ord('s'):
        fname = f"frame_{int(current_t)}.jpg"
        cv.imwrite(fname, frame)
        print(f"[INFO] Saved {fname}")

# --- SHUTDOWN ---
send_to_rover("STOP")
print("[INFO] Rover stopped. Shutting down.")
cap.release()
cv.destroyAllWindows()
sock.close()
