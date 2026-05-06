import math
import struct
import time

import cv2
import torch

try:
    import serial
except ImportError:
    serial = None

from ultralytics import YOLO

# 模型与标签配置
model = YOLO("6_ncnn_model", task="detect")
REQUIRED_LABELS = {"apple", "banana", "circle", "square", "strawberry", "trapezoid", "triangle", "watermelon"}
FULL_LABEL_SET = {"apple", "banana", "square", "strawberry", "trapezoid", "triangle", "watermelon"}
TARGET_LABELS = REQUIRED_LABELS | {"bananna"}
LABEL_ALIAS = {"bananna": "banana"}

# 跟踪参数
STABLE_FRAMES = 1
MATCH_DISTANCE = 80

# 串口协议
SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_BAUDRATE = 921600
SERIAL_TIMEOUT = 0.1
CIRCLE_CMD_ID = 0x0101
CIRCLE_FLAGS = 0x0001
LASER_POINT = (777, 582)
SEND_INTERVAL = 0.05
last_send_time = 0

MODE_WAIT = 0
MODE_AUTO_CYCLE = 1
MODE_CIRCLE_PATH = 2
MODE_LOCK_CIRCLE = 3
MODE_LOCK_ORDER_CYCLE = 4
current_mode = MODE_WAIT

has_finished_cycle = False

# 圆周运动参数
CIRCLE_RADIUS = 15.0
THETA_STEP = 0.03
circle_theta = 0.0

# Circle 平滑
SMOOTH_FACTOR = 0.35
smooth_circle_x = None
smooth_circle_y = None

SWITCH_INTERVAL = 15  # 物体切换间隔 15秒

# 全局状态
serial_port = None
serial_enabled = False
auto_mode = False
obj_list = []
cur_obj_idx = 0
last_switch_t = 0
cycle_finished = False

label_to_id = {
    "apple": 1,        # 苹果
    "banana": 2,       # 香蕉
    "square": 3,       # 正方形
    "strawberry": 4,   # 草莓
    "trapezoid": 5,    # 梯形
    "triangle": 6,     # 三角形
    "watermelon": 7,   # 西瓜
    "circle": 8        # 圆
}

def normalize_label(label):
    return LABEL_ALIAS.get(label, label)


def center_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def create_track(tid, det):
    return {"id": tid, "label": det["label"], "center": det["center"], "hits": 1, "missed": 0}


def update_track(track, det):
    track["center"] = det["center"]
    track["hits"] += 1
    track["missed"] = 0


def open_camera():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened(): cap = cv2.VideoCapture(0)
    return cap


def init_serial():
    global serial_port, serial_enabled
    if serial is None:
        serial_enabled = False
        return False
    if serial_port is not None and serial_port.is_open:
        serial_enabled = True
        return True
    try:
        serial_port = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
        serial_enabled = True
        return True
    except:
        serial_enabled = False
        return False


def close_serial():
    global serial_port, serial_enabled
    if serial_port is not None and serial_port.is_open:
        serial_port.close()
    serial_port = None
    serial_enabled = False


def pack_frame(cmd_id, flags, floats):
    n = len(floats)
    length = 2 + 2 + 4 * n
    buf = bytearray()
    buf.append(0xA5)
    buf.append(length & 0xFF)
    buf += struct.pack("<H", cmd_id)
    buf += struct.pack("<H", flags)
    for v in floats:
        buf += struct.pack("<f", v)
    return bytes(buf)


def calc_err(center):
    dx = LASER_POINT[0] - center[0]
    dy = LASER_POINT[1] - center[1]
    return float(dx), float(dy), 1000.0


# 发送函数：物体不稳定 → dx=0, dy=0，obj_id 保持不变
def send_target_data_stable(current_track, last_obj_id):
    global last_send_time
    now = time.time()
    if now - last_send_time < SEND_INTERVAL:
        return
    last_send_time = now
    if not init_serial():
        return

    # ===================== 核心逻辑 =====================
    # 物体稳定 → 正常发送 dx, dy
    # 物体丢失/不稳定 → dx=0, dy=0，obj_id 保持不变
    # ====================================================
    if current_track is not None and current_track["hits"] >= STABLE_FRAMES:
        dx, dy, dist = calc_err(current_track["center"])
        obj_id = label_to_id.get(current_track["label"], 0)
    else:
        dx = 0.0
        dy = 0.0
        obj_id = last_obj_id  # 保留原来的ID

    payload = [
        float(dx),
        float(dy),
        1000.0,
        float(obj_id),
        float(time.time()),
        0.0,0.0,0.0,0.0,0.0,0.0,0.0
    ]

    try:
        frame = pack_frame(CIRCLE_CMD_ID, CIRCLE_FLAGS, payload)
        serial_port.write(frame)
        if current_track is not None and current_track["hits"] >= STABLE_FRAMES:
            print(f"发送 -> {current_track['label']} ID:{obj_id} | dx={dx:.0f} dy={dy:.0f}")
        else:
            print(f"发送 -> 【目标不稳定】dx=0 dy=0 | 保留ID:{obj_id}")
    except:
        pass


def listen_serial():
    if not serial_enabled or serial_port is None or not serial_port.is_open:
        return ""
    try:
        if serial_port.in_waiting > 0:
            data = serial_port.read(serial_port.in_waiting)
            return data.hex(" ")
    except:
        pass
    return ""


# 初始化摄像头
cap = open_camera()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

tracks = {}
next_tid = 0
last_msg = ""
last_obj_id = 0  # 保存上一次的物体ID

while True:
    ret, frame = cap.read()
    if not ret: break

    # 目标检测
    res = model.predict(frame, imgsz=640, conf=0.6, verbose=False)[0]
    frame_draw = res.plot()
    dets = []
    if res.boxes is not None and len(res.boxes) > 0:
        for box in res.boxes:
            # 过滤无效框 NaN
            xyxy = box.xyxy[0]
            if any(torch.isnan(xyxy)):
                continue

            x1, y1, x2, y2 = xyxy.tolist()
            if x1 <= 0 or y1 <= 0 or x2 <= 0 or y2 <= 0:
                continue

            lab = normalize_label(model.names[int(box.cls[0])])
            if lab not in TARGET_LABELS:
                continue

            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            dets.append({"label": lab, "center": (cx, cy)})

    # 目标跟踪
    matched = set()
    matched_det = set()
    for di, d in enumerate(dets):
        best_tid = None
        best_d = 9999
        for tid, t in tracks.items():
            if tid in matched or t["label"] != d["label"]:
                continue
            dist = center_distance(t["center"], d["center"])
            if dist < MATCH_DISTANCE and dist < best_d:
                best_d = dist
                best_tid = tid
        if best_tid is not None:
            update_track(tracks[best_tid], d)
            matched.add(best_tid)
            matched_det.add(di)
    for di, d in enumerate(dets):
        if di not in matched_det:
            tracks[next_tid] = create_track(next_tid, d)
            matched.add(next_tid)
            next_tid += 1
    # 清理丢失目标
    del_list = []
    for tid, t in tracks.items():
        if tid not in matched:
            t["missed"] += 1
            if t["missed"] > 10:
                del_list.append(tid)
    for tid in del_list:
        del tracks[tid]

    # 收集稳定目标
    stable = []
    circle_raw = None
    for t in tracks.values():
        if t["hits"] >= STABLE_FRAMES:
            stable.append(t)
            if t["label"] == "circle":
                circle_raw = t["center"]

    # Circle 平滑
    circle_smooth = None
    if circle_raw:
        rx, ry = circle_raw
        if smooth_circle_x is None:
            smooth_circle_x, smooth_circle_y = rx, ry
        else:
            smooth_circle_x = smooth_circle_x * (1 - SMOOTH_FACTOR) + rx * SMOOTH_FACTOR
            smooth_circle_y = smooth_circle_y * (1 - SMOOTH_FACTOR) + ry * SMOOTH_FACTOR
        circle_smooth = (int(smooth_circle_x), int(smooth_circle_y))

    # 串口监听 & 触发
    recv = listen_serial()
    if recv and len(recv) > 0:
        print(f"收到串口：{recv}")
        if "01" in recv:
            current_mode = MODE_AUTO_CYCLE
            auto_mode = True  # 保持兼容
            print("【模式切换】自动轮发")
        elif "02" in recv:
            current_mode = MODE_CIRCLE_PATH
            auto_mode = False
            print("【模式切换】圆周运动")
        elif "03" in recv:
            current_mode = MODE_LOCK_CIRCLE
            print("【模式切换】锁定圆心")
        elif "04" in recv:
            current_mode = MODE_LOCK_ORDER_CYCLE
            cur_obj_idx = 0  # 从头开始
            last_switch_t = time.time()
            cycle_finished = False
            print("【模式切换】抓拍顺序固定轮询")
        elif "00" in recv:
            current_mode = MODE_WAIT
            auto_mode = False
            print("【模式切换】等待")

    current_selected_track = None

    if current_mode == MODE_AUTO_CYCLE:
        valid_stable = [t for t in stable if t["label"] != "circle"]
        current_labels = {t["label"] for t in valid_stable}

        # 初始必须集齐7种才开始
        if not has_finished_cycle:
            if not FULL_LABEL_SET.issubset(current_labels):
                current_selected_track = None
                print(f"01等待集齐7物：{len(current_labels)}/7")
            else:
                # 集齐初始化
                has_finished_cycle = True
                cur_obj_idx = 0
                last_switch_t = time.time()
                print("01 集齐7物，开始轮询")

        # 已集齐，开始轮询
        if has_finished_cycle:
            obj_list = sorted(valid_stable, key=lambda x: x["center"][0])
            list_len = len(obj_list)

            if list_len == 0:
                current_selected_track = None
            else:
                # 索引保护
                if cur_obj_idx >= list_len:
                    cur_obj_idx = list_len - 1

                # 当前目标（必须赋值，才会输出dx/dy）
                current_target = obj_list[cur_obj_idx]
                current_track = tracks.get(current_target["label"], None)
                current_selected_track = current_track

                # 核心逻辑：区分“是否为最后一个”
                is_last = (cur_obj_idx == list_len - 1)

                if is_last:
                    # --- 逻辑分支 A：已经是最后一个，只计时，不切换 ---
                    if time.time() - last_switch_t >= SWITCH_INTERVAL:
                        current_mode = MODE_WAIT
                        cur_obj_idx = 0
                        has_finished_cycle = False
                        # obj_list.clear()  # 清空列表以防残留
                        print("01 一轮轮询完成，自动停止")
                else:
                    # --- 逻辑分支 B：还有后续目标，尝试切换 ---
                    next_idx = cur_obj_idx + 1
                    next_target = obj_list[next_idx]
                    next_track = tracks.get(next_target["label"], None)

                    # 只有时间到了且下一个目标稳定，才执行切换
                    if time.time() - last_switch_t >= SWITCH_INTERVAL:
                        if next_track is not None and next_track["hits"] >= STABLE_FRAMES:
                            cur_obj_idx = next_idx
                            last_switch_t = time.time()
                            print(f"01 切换 → {next_target['label']}")

    elif current_mode == MODE_CIRCLE_PATH:
        if circle_smooth:
            if circle_theta < 1.2 * 2 * math.pi:
                circle_theta += THETA_STEP
                tx = int(circle_smooth[0] + CIRCLE_RADIUS * math.cos(circle_theta))
                ty = int(circle_smooth[1] + CIRCLE_RADIUS * math.sin(circle_theta))
                current_selected_track = {"label": "circle", "center": (tx, ty), "hits": STABLE_FRAMES}
                cv2.circle(frame_draw, (tx, ty), 8, (0, 255, 0), 2)
            else:
                # 1.2周完成，重置角度并切换到等待
                circle_theta = 0.0
                current_mode = MODE_WAIT
                print("画圆 1.2 周结束")

    elif current_mode == MODE_LOCK_CIRCLE:
        if circle_smooth:
            # 直接将圆心作为目标点
            current_selected_track = {
                "label": "circle",
                "center": circle_smooth,
                "hits": STABLE_FRAMES
            }
            # 视觉反馈：在圆心绘制一个明显的锁定框
            cv2.circle(frame_draw, circle_smooth, 15, (0, 255, 255), 2)
        else:
            current_selected_track = None
            cv2.putText(frame_draw, "Targeting Circle...", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    elif current_mode == MODE_LOCK_ORDER_CYCLE:
        if len(obj_list) == 0:
            current_selected_track = None
            print("请先运行 01 模式")
            continue

        list_len = len(obj_list)
        if cur_obj_idx >= list_len:
            cur_obj_idx = 0

        current_target = obj_list[cur_obj_idx]
        current_track = tracks.get(current_target["label"], None)
        current_selected_track = current_track

        next_idx = cur_obj_idx + 1
        if next_idx >= list_len:
            current_mode = MODE_WAIT
            cur_obj_idx = 0
            last_obj_id = 99
            print("04 一轮完成")
            current_selected_track = None
        else:
            next_target = obj_list[next_idx]
            next_track = tracks.get(next_target["label"], None)

            # 15秒到 + 下一个稳定 → 切换
            if time.time() - last_switch_t >= SWITCH_INTERVAL:
                if next_track is not None and next_track["hits"] >= STABLE_FRAMES:
                    cur_obj_idx = next_idx
                    last_switch_t = time.time()
                    print(f"04 切换到：{next_target['label']}")

    # ======================================================

    # 更新最后一次有效ID
    if current_selected_track is not None and current_selected_track["hits"] >= STABLE_FRAMES:
        last_obj_id = label_to_id.get(current_selected_track["label"], last_obj_id)

    # 发送数据
    send_target_data_stable(current_selected_track, last_obj_id)

    # 绘制
    for t in stable:
        cv2.circle(frame_draw, t["center"], 5, (0, 0, 255), -1)
    cv2.circle(frame_draw, LASER_POINT, 6, (255, 0, 0), 2)

    mode_names = {0: "Wait", 1: "Auto Cycle", 2: "Circle Path", 3: "Lock Circle"}
    mode_txt = f"Mode: {mode_names.get(current_mode, 'Unknown')}"
    cv2.putText(frame_draw, mode_txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("Detect", frame_draw)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
    if key == ord("c"):
        tracks.clear()
        auto_mode = False
        obj_list.clear()
        smooth_circle_x = smooth_circle_y = None
        last_obj_id = 0
        cur_obj_idx = 0
        cycle_finished = False

cap.release()
close_serial()
cv2.destroyAllWindows()