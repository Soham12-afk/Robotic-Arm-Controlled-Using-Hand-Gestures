"""
ROBOTIC ARM CONTROLLER 
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
GRID LAYOUT:
  [ ELBOW UP  ] [ SHOULDER UP ] [ ELBOW DOWN ]
  [ BASE LEFT ] [  CLAW ZONE  ] [ BASE RIGHT ]
  [ WRIST UP  ] [ SHOULDER DN ] [ WRIST DOWN ]

12 COMMANDS:
  ZONE COMMANDS (move hand to zone):
    ELBOW UP    → P    ELBOW DOWN  → p
    SHOULDER UP → C    SHOULDER DN → c
    WRIST UP    → U    WRIST DOWN  → G
    BASE LEFT   → S    BASE RIGHT  → O

  ZONE + FIST (close all fingers in zone):
    BASE LEFT + fist  → L  (wrist2 CCW)
    BASE RIGHT + fist → R  (wrist2 CW)

  CENTER ZONE (claw only):
    Open hand  → F  (claw open)
    Fist       → f  (claw close)

SET ESP32_IP below before running.
"""
import cv2
import socket
import time
import collections
import numpy as np
from mediapipe.python.solutions import hands as mp_hands
from mediapipe.python.solutions import drawing_utils as mp_draw

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# CONFIG
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ESP32_IP        = "10.105.159.53"  # ← YOUR ESP32 IP
ESP32_PORT      = 4210 

# MUST be > ESP32 CMD_DEBOUNCE_MS (250ms) so every send gets through
# MUST be < how long user holds hand in zone for repeated moves
SEND_DELAY      = 0.35             # 350ms between sends of same command

SMOOTH_N        = 3                # frames to smooth hand position
FIST_THRESHOLD  = 4                # fingers closed to count as fist

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# ZONE MAP
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ZONE_LABELS = {
    (0,0): "ELBOW UP",   (1,0): "SHOULDER UP", (2,0): "ELBOW DOWN",
    (0,1): "BASE LEFT",  (1,1): "CLAW ZONE",   (2,1): "BASE RIGHT",
    (0,2): "WRIST UP",   (1,2): "SHOULDER DN", (2,2): "WRIST DOWN",
}

ZONE_OPEN_CMD = {
    (0,0): "P",  (1,0): "C",  (2,0): "p",
    (0,1): "S",  (1,1): "F",  (2,1): "O",
    (0,2): "U",  (1,2): "c",  (2,2): "G",
}

ZONE_FIST_CMD = {
    (0,1): "L",
    (2,1): "R",
    (1,1): "f",
}

CMD_DESC = {
    "P": "ELBOW UP",    "p": "ELBOW DOWN",
    "C": "SHLDR UP",    "c": "SHLDR DOWN",
    "U": "WRIST UP",    "G": "WRIST DOWN",
    "S": "BASE LEFT",   "O": "BASE RIGHT",
    "F": "CLAW OPEN",   "f": "CLAW CLOSE",
    "L": "WRIST2 CCW",  "R": "WRIST2 CW",
}

COL = {
    "grid":     (40,  40,  55),
    "active":   (0,  220, 120),
    "fist":     (0,  120, 220),
    "center":   (30, 100, 180),
    "text":     (200,200, 210),
    "cmd":      (0,  255, 100),
    "warn":     (0,  120, 255),
    "hand_dot": (255, 80,   0),
}

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# UDP
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def send_udp(cmd):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.sendto(cmd.encode(), (ESP32_IP, ESP32_PORT))
        s.close()
    except Exception as e:
        print(f"UDP error: {e}")

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# HAND DETECTION
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
FINGER_TIPS = [8, 12, 16, 20]
FINGER_PIPS = [6, 10, 14, 18]

def get_finger_states(lm, hand_label):
    states = []
    for tip, pip in zip(FINGER_TIPS, FINGER_PIPS):
        states.append(1 if lm[tip].y > lm[pip].y else 0)
    if hand_label == "Right":
        states.append(1 if lm[4].x < lm[3].x else 0)
    else:
        states.append(1 if lm[4].x > lm[3].x else 0)
    return states

def is_fist(fingers):
    return sum(fingers) >= FIST_THRESHOLD

def get_zone(cx, cy, w, h):
    col = min(int(cx / (w / 3)), 2)
    row = min(int(cy / (h / 3)), 2)
    return col, row

def resolve_command(zone, fingers):
    if is_fist(fingers):
        return ZONE_FIST_CMD.get(zone, None)
    else:
        return ZONE_OPEN_CMD.get(zone, None)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# HUD
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def draw_overlay(frame, active_zone, command, fingers, hand_label, fps, send_count):
    h, w   = frame.shape[:2]
    cw, rh = w // 3, h // 3
    fist   = is_fist(fingers)

    overlay = frame.copy()
    for row in range(3):
        for col in range(3):
            zone = (col, row)
            x1, y1 = col * cw, row * rh
            is_active = zone == active_zone
            is_center = zone == (1, 1)
            fill = None
            if is_active and fist:
                fill = (20, 20, 80)
            elif is_active:
                fill = (0, 60, 30)
            elif is_center:
                fill = (10, 30, 60)
            if fill:
                cv2.rectangle(overlay, (x1, y1), (x1+cw, y1+rh), fill, -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

    for i in range(1, 3):
        cv2.line(frame, (i*cw, 0), (i*cw, h),  COL["grid"], 1)
        cv2.line(frame, (0, i*rh), (w, i*rh),  COL["grid"], 1)

    for row in range(3):
        for col in range(3):
            zone  = (col, row)
            label = ZONE_LABELS[zone]
            x1, y1 = col * cw, row * rh
            is_active = zone == active_zone
            is_center = zone == (1, 1)

            border = COL["fist"] if (is_active and fist) else \
                     COL["active"] if is_active else \
                     COL["center"] if is_center else COL["grid"]
            thickness = 2 if is_active else 1
            cv2.rectangle(frame, (x1+1, y1+1), (x1+cw-1, y1+rh-1), border, thickness)

            fs = 0.4
            tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, 1)[0][0]
            tx = x1 + (cw - tw) // 2
            ty = y1 + rh // 2
            cv2.putText(frame, label, (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, border, 1, cv2.LINE_AA)

            oc = ZONE_OPEN_CMD.get(zone, "")
            cv2.putText(frame, f"open→{oc}", (x1+4, y1+rh-22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (80,80,100), 1)

            fc = ZONE_FIST_CMD.get(zone, "")
            if fc:
                cv2.putText(frame, f"fist→{fc}", (x1+4, y1+rh-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, (80,80,160), 1)

    bar_y = h - 50
    cv2.rectangle(frame, (0, bar_y), (w, h), (8, 8, 14), -1)
    cv2.line(frame, (0, bar_y), (w, bar_y), COL["grid"], 1)

    desc    = CMD_DESC.get(command, "---")
    cmd_str = f"CMD: {command or '--'}  [{desc}]"
    cv2.putText(frame, cmd_str, (12, bar_y+18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, COL["cmd"], 2, cv2.LINE_AA)

    hand_state = "FIST" if fist else "OPEN"
    state_col  = COL["fist"] if fist else COL["active"]
    cv2.putText(frame, f"Hand:{hand_state} Sends:{send_count}", (12, bar_y+38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, state_col, 1)

    f_str = "".join(["█" if f else "░" for f in fingers])
    cv2.putText(frame, f"Fngr:{f_str}", (w//2-55, bar_y+18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160,160,180), 1)
    cv2.putText(frame, "I M R P T", (w//2-55, bar_y+36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (80,80,100), 1)

    cv2.putText(frame, f"ESP32:{ESP32_IP}", (w-230, bar_y+18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, COL["active"], 1)
    cv2.putText(frame, f"FPS:{fps:.0f}  {hand_label or '--'}",
                (w-230, bar_y+36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, COL["text"], 1)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# MAIN
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def main():
    last_sent_cmd  = None
    last_send_time = 0
    send_count     = 0          # tracks actual UDP sends — compare with arm moves

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    cx_buf = collections.deque(maxlen=SMOOTH_N)
    cy_buf = collections.deque(maxlen=SMOOTH_N)
    fps_t  = time.time()
    fps_cnt = 0
    fps     = 0

    print("━" * 55)
    print("  ROBOTIC ARM CONTROLLER  |  ESC to quit")
    print(f"  ESP32 → {ESP32_IP}:{ESP32_PORT}")
    print(f"  Send delay: {SEND_DELAY*1000:.0f}ms  (ESP32 debounce: 250ms)")
    print("━" * 55)
    for cmd, desc in CMD_DESC.items():
        print(f"    {cmd}  →  {desc}")
    print("━" * 55)

    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.75,
        min_tracking_confidence=0.65,
    ) as detector:

        while cap.isOpened():
            ok, frame = cap.read()
            if not ok:
                break

            frame = cv2.flip(frame, 1)
            h, w  = frame.shape[:2]
            rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res   = detector.process(rgb)

            fps_cnt += 1
            if time.time() - fps_t >= 1.0:
                fps     = fps_cnt
                fps_cnt = 0
                fps_t   = time.time()

            command     = None
            active_zone = None
            hand_label  = None
            fingers     = [0, 0, 0, 0, 0]

            if res.multi_hand_landmarks:
                lm_list    = res.multi_hand_landmarks[0].landmark
                hand_label = res.multi_handedness[0].classification[0].label

                mp_draw.draw_landmarks(
                    frame,
                    res.multi_hand_landmarks[0],
                    mp_hands.HAND_CONNECTIONS,
                )

                raw_cx = int(lm_list[9].x * w)
                raw_cy = int(lm_list[9].y * h)
                cx_buf.append(raw_cx)
                cy_buf.append(raw_cy)
                cx = int(np.mean(cx_buf))
                cy = int(np.mean(cy_buf))

                cv2.circle(frame, (cx, cy), 10, COL["hand_dot"], -1)
                cv2.circle(frame, (cx, cy), 14, COL["hand_dot"],  1)

                fingers     = get_finger_states(lm_list, hand_label)
                active_zone = get_zone(cx, cy, w, h)
                command     = resolve_command(active_zone, fingers)

                if command:
                    now = time.time()
                    # Send if: new command (immediate) OR same cmd held long enough
                    if command != last_sent_cmd or (now - last_send_time) >= SEND_DELAY:
                        send_udp(command)
                        send_count    += 1
                        last_sent_cmd  = command
                        last_send_time = now

                        # Console print every actual send — matches arm moves exactly
                        ts   = time.strftime("%H:%M:%S")
                        desc = CMD_DESC.get(command, "?")
                        f_str = "".join(["1" if f else "0" for f in fingers])
                        print(f"{ts}  #{send_count:04d}  {command}  {desc:<14}  [{f_str}]")

            else:
                last_sent_cmd = None   # reset so next detection fires immediately
                cv2.putText(frame, "NO HAND DETECTED", (w//2-120, h//2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, COL["warn"], 2)

            draw_overlay(frame, active_zone, command, fingers, hand_label, fps, send_count)
            cv2.imshow("Robotic Arm Controller | ESC=Quit", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    cap.release()
    cv2.destroyAllWindows()
    print(f"Stopped. Total sends: {send_count}")

if __name__ == "__main__":
    main()