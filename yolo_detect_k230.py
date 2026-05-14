import os,sys,gc,math
from media.media import *
from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import nncase_runtime as nn
import ulab.numpy as np
import image

# ==================== 标签配置 ====================
REQUIRED_LABELS = {"apple", "banana", "circle", "square", "strawberry", "trapezoid", "triangle", "watermelon"}
FULL_LABEL_SET = {"apple", "banana", "square", "strawberry", "trapezoid", "triangle", "watermelon"}
TARGET_LABELS = REQUIRED_LABELS | {"bananna"}
LABEL_ALIAS = {"bananna": "banana"}

# ==================== 标签ID映射 ====================
label_to_id = {
    "apple": 1,
    "banana": 2,
    "square": 3,
    "strawberry": 4,
    "trapezoid": 5,
    "triangle": 6,
    "watermelon": 7,
    "circle": 8
}

LABEL_NAMES = {
    0: "apple",
    1: "banana",
    2: "circle",
    3: "square",
    4: "strawberry",
    5: "trapezoid",
    6: "triangle",
    7: "watermelon"
}

# ==================== 跟踪参数 ====================
STABLE_FRAMES = 1
MATCH_DISTANCE = 80

# ==================== 圆周运动参数 ====================
CIRCLE_RADIUS = 15.0
THETA_STEP = 0.03
circle_theta = 0.0

# ==================== Circle平滑 ====================
SMOOTH_FACTOR = 0.35
smooth_circle_x = None
smooth_circle_y = None

# ==================== 切换间隔 ====================
SWITCH_INTERVAL = 15

# ==================== 模式定义 ====================
MODE_WAIT = 0
MODE_AUTO_CYCLE = 1
MODE_CIRCLE_PATH = 2
MODE_LOCK_CIRCLE = 3
MODE_LOCK_ORDER_CYCLE = 4

# ==================== 串口协议 ====================
CIRCLE_CMD_ID = 0x0101
CIRCLE_FLAGS = 0x0001

# ==================== 激光点标定坐标 ====================
LASER_POINT = (640, 360)

# ==================== 发送间隔 ====================
SEND_INTERVAL = 0.05

# ==================== 全局状态 ====================
current_mode = MODE_WAIT
has_finished_cycle = False
serial_port = None
serial_enabled = False
obj_list = []
cur_obj_idx = 0
last_switch_t = 0
cycle_finished = False
last_obj_id = 0
last_send_time = 0
tracks = {}
next_tid = 0

# ==================== 串口相关 ====================
try:
    from machine import UART
except ImportError:
    UART = None


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


# ==================== YOLO检测类 ====================
class YoloDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, labels_map, confidence_threshold=0.6,
                 rgb888p_size=[1280, 720], display_size=[640, 360], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.labels_map = labels_map
        self.confidence_threshold = confidence_threshold
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, _ = letterbox_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                           [1, 3, self.model_input_size[1], self.model_input_size[0]])

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            dets = []
            if len(results) > 0:
                output = results[0]
                if isinstance(output, nn.Tensor):
                    output = output.to_numpy()
                dets = self.parse_yolo_output(output)
            return dets

    def parse_yolo_output(self, output):
        dets = []
        try:
            output = output.reshape(output.shape[0], -1)
            for det in output:
                if len(det) < 6:
                    continue
                x1, y1, x2, y2, score, cls_id = det[:6]
                if score < self.confidence_threshold:
                    continue
                cls_id = int(cls_id)
                label = self.labels_map.get(cls_id, None)
                if label is None or label not in TARGET_LABELS:
                    continue
                cx = int((x1 + x2) // 2)
                cy = int((y1 + y2) // 2)
                dets.append({
                    "label": normalize_label(label),
                    "center": (cx, cy),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "score": float(score)
                })
        except Exception as e:
            print("[Parse Error]", e)
        return dets

    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                for det in dets:
                    x1, y1, x2, y2 = det["bbox"]
                    label = det["label"]
                    cx, cy = det["center"]

                    x1_disp = x1 * self.display_size[0] // self.rgb888p_size[0]
                    y1_disp = y1 * self.display_size[1] // self.rgb888p_size[1]
                    x2_disp = x2 * self.display_size[0] // self.rgb888p_size[0]
                    y2_disp = y2 * self.display_size[1] // self.rgb888p_size[1]

                    pl.osd_img.draw_rectangle(x1_disp, y1_disp, x2_disp - x1_disp, y2_disp - y1_disp,
                                             color=(255, 255, 0, 255), thickness=2)
                    pl.osd_img.draw_string(x1_disp, y1_disp - 20, label, color=(0, 255, 0, 255))
                    pl.osd_img.draw_circle(cx, cy, 5, color=(0, 0, 255), fill=True)
            else:
                pl.osd_img.clear()


# ==================== 串口函数 ====================
def pack_frame(cmd_id, flags, floats):
    import struct
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


def send_target_data_stable(current_track, last_obj_id):
    global last_send_time

    now = time.time()
    if now - last_send_time < SEND_INTERVAL:
        return
    last_send_time = now

    if not serial_enabled or serial_port is None:
        return

    if current_track is not None and current_track["hits"] >= STABLE_FRAMES:
        dx, dy, dist = calc_err(current_track["center"])
        obj_id = label_to_id.get(current_track["label"], 0)
    else:
        dx = 0.0
        dy = 0.0
        obj_id = last_obj_id

    payload = [
        float(dx), float(dy), 1000.0, float(obj_id),
        float(time.time()), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ]

    try:
        frame_data = pack_frame(CIRCLE_CMD_ID, CIRCLE_FLAGS, payload)
        serial_port.write(frame_data)
        if current_track is not None and current_track["hits"] >= STABLE_FRAMES:
            print("发送 -> {} ID:{} | dx={:.0f} dy={:.0f}".format(
                current_track["label"], obj_id, dx, dy))
        else:
            print("发送 -> 【目标不稳定】dx=0 dy=0 | 保留ID:{}".format(obj_id))
    except:
        pass


def listen_serial():
    if not serial_enabled or serial_port is None:
        return ""
    try:
        if serial_port.any():
            data = serial_port.read()
            return data.hex(" ")
    except:
        pass
    return ""


def init_serial():
    global serial_port, serial_enabled
    if UART is None:
        serial_enabled = False
        return
    try:
        serial_port = UART(1, 921600, timeout=0.1)
        serial_enabled = True
        print("[UART] 串口初始化成功")
    except Exception as e:
        serial_enabled = False
        print("[UART] 串口初始化失败:", e)


# ==================== 主程序 ====================
if __name__ == "__main__":
    import time

    display_mode = "lcd"
    display_size = None
    rgb888p_size = [1280, 720]
    kmodel_path = "/sdcard/yolo.kmodel"
    model_input_size = [640, 640]
    confidence_threshold = 0.6
    debug_mode = 0

    print("[Main] 初始化PipeLine...")
    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode, display_size=display_size)
    pl.create()
    display_size = pl.get_display_size()

    print("[Main] 初始化YOLO模型...")
    yolo_det = YoloDetectionApp(
        kmodel_path=kmodel_path,
        model_input_size=model_input_size,
        labels_map=LABEL_NAMES,
        confidence_threshold=confidence_threshold,
        rgb888p_size=rgb888p_size,
        display_size=display_size,
        debug_mode=debug_mode
    )
    yolo_det.config_preprocess()

    print("[Main] 初始化串口...")
    init_serial()

    print("[Main] 开始主循环")

    while True:
        with ScopedTiming("total", debug_mode > 0):
            img = pl.get_frame()
            dets = yolo_det.run(img)

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

            del_list = []
            for tid, t in tracks.items():
                if tid not in matched:
                    t["missed"] += 1
                    if t["missed"] > 10:
                        del_list.append(tid)
            for tid in del_list:
                del tracks[tid]

            stable = []
            circle_raw = None
            for t in tracks.values():
                if t["hits"] >= STABLE_FRAMES:
                    stable.append(t)
                    if t["label"] == "circle":
                        circle_raw = t["center"]

            circle_smooth = None
            if circle_raw:
                rx, ry = circle_raw
                if smooth_circle_x is None:
                    smooth_circle_x, smooth_circle_y = rx, ry
                else:
                    smooth_circle_x = smooth_circle_x * (1 - SMOOTH_FACTOR) + rx * SMOOTH_FACTOR
                    smooth_circle_y = smooth_circle_y * (1 - SMOOTH_FACTOR) + ry * SMOOTH_FACTOR
                circle_smooth = (int(smooth_circle_x), int(smooth_circle_y))

            recv = listen_serial()
            if recv and len(recv) > 0:
                print("收到串口：{}".format(recv))
                if "01" in recv:
                    current_mode = MODE_AUTO_CYCLE
                    print("【模式切换】自动轮发")
                elif "02" in recv:
                    current_mode = MODE_CIRCLE_PATH
                    print("【模式切换】圆周运动")
                elif "03" in recv:
                    current_mode = MODE_LOCK_CIRCLE
                    print("【模式切换】锁定圆心")
                elif "04" in recv:
                    current_mode = MODE_LOCK_ORDER_CYCLE
                    cur_obj_idx = 0
                    last_switch_t = time.time()
                    cycle_finished = False
                    print("【模式切换】抓拍顺序固定轮询")
                elif "00" in recv:
                    current_mode = MODE_WAIT
                    print("【模式切换】等待")

            current_selected_track = None

            if current_mode == MODE_AUTO_CYCLE:
                valid_stable = [t for t in stable if t["label"] != "circle"]
                current_labels = {t["label"] for t in valid_stable}

                if not has_finished_cycle:
                    if not FULL_LABEL_SET.issubset(current_labels):
                        current_selected_track = None
                        print("01等待集齐7物：{}/7".format(len(current_labels)))
                    else:
                        has_finished_cycle = True
                        cur_obj_idx = 0
                        last_switch_t = time.time()
                        print("01 集齐7物，开始轮询")

                if has_finished_cycle:
                    obj_list = sorted(valid_stable, key=lambda x: x["center"][0])
                    list_len = len(obj_list)

                    if list_len == 0:
                        current_selected_track = None
                    else:
                        if cur_obj_idx >= list_len:
                            cur_obj_idx = list_len - 1

                        current_target = obj_list[cur_obj_idx]
                        current_track = None
                        for t in tracks.values():
                            if t["label"] == current_target["label"]:
                                current_track = t
                                break
                        current_selected_track = current_track

                        is_last = (cur_obj_idx == list_len - 1)

                        if is_last:
                            if time.time() - last_switch_t >= SWITCH_INTERVAL:
                                current_mode = MODE_WAIT
                                cur_obj_idx = 0
                                has_finished_cycle = False
                                print("01 一轮轮询完成，自动停止")
                        else:
                            next_idx = cur_obj_idx + 1
                            next_target = obj_list[next_idx]
                            next_track = None
                            for t in tracks.values():
                                if t["label"] == next_target["label"]:
                                    next_track = t
                                    break

                            if time.time() - last_switch_t >= SWITCH_INTERVAL:
                                if next_track is not None and next_track["hits"] >= STABLE_FRAMES:
                                    cur_obj_idx = next_idx
                                    last_switch_t = time.time()
                                    print("01 切换 -> {}".format(next_target["label"]))

            elif current_mode == MODE_CIRCLE_PATH:
                if circle_smooth:
                    if circle_theta < 1.2 * 2 * math.pi:
                        circle_theta += THETA_STEP
                        tx = int(circle_smooth[0] + CIRCLE_RADIUS * math.cos(circle_theta))
                        ty = int(circle_smooth[1] + CIRCLE_RADIUS * math.sin(circle_theta))
                        current_selected_track = {"label": "circle", "center": (tx, ty), "hits": STABLE_FRAMES}
                        pl.osd_img.draw_circle(tx, ty, 8, color=(0, 255, 0, 255), thickness=2)
                    else:
                        circle_theta = 0.0
                        current_mode = MODE_WAIT
                        print("画圆 1.2 周结束")

            elif current_mode == MODE_LOCK_CIRCLE:
                if circle_smooth:
                    current_selected_track = {
                        "label": "circle",
                        "center": circle_smooth,
                        "hits": STABLE_FRAMES
                    }
                    pl.osd_img.draw_circle(circle_smooth[0], circle_smooth[1], 15, color=(0, 255, 255, 255), thickness=2)
                else:
                    current_selected_track = None
                    pl.osd_img.draw_string(20, 80, "Targeting Circle...", color=(0, 0, 255, 255))

            elif current_mode == MODE_LOCK_ORDER_CYCLE:
                if len(obj_list) == 0:
                    current_selected_track = None
                    print("请先运行 01 模式")
                else:
                    list_len = len(obj_list)
                    if cur_obj_idx >= list_len:
                        cur_obj_idx = 0

                    current_target = obj_list[cur_obj_idx]
                    current_track = None
                    for t in tracks.values():
                        if t["label"] == current_target["label"]:
                            current_track = t
                            break
                    current_selected_track = current_track

                    is_last = (cur_obj_idx == list_len - 1)
                    if is_last:
                        if time.time() - last_switch_t >= SWITCH_INTERVAL:
                            current_mode = MODE_WAIT
                            cur_obj_idx = 0
                            last_obj_id = 99
                            cycle_finished = True
                            print("04 一轮完成")
                    else:
                        next_idx = cur_obj_idx + 1
                        next_target = obj_list[next_idx]
                        next_track = None
                        for t in tracks.values():
                            if t["label"] == next_target["label"]:
                                next_track = t
                                break

                        if time.time() - last_switch_t >= SWITCH_INTERVAL:
                            if next_track is not None and next_track["hits"] >= STABLE_FRAMES:
                                cur_obj_idx = next_idx
                                last_switch_t = time.time()
                                print("04 切换到：{}".format(next_target["label"]))

            if current_selected_track is not None and current_selected_track["hits"] >= STABLE_FRAMES:
                last_obj_id = label_to_id.get(current_selected_track["label"], last_obj_id)

            send_target_data_stable(current_selected_track, last_obj_id)

            for t in stable:
                pl.osd_img.draw_circle(t["center"][0], t["center"][1], 5, color=(0, 0, 255, 255), fill=True)

            pl.osd_img.draw_circle(LASER_POINT[0], LASER_POINT[1], 6, color=(255, 0, 0, 255), thickness=2)

            mode_names = {0: "Wait", 1: "Auto Cycle", 2: "Circle Path", 3: "Lock Circle", 4: "Lock Order"}
            mode_txt = "Mode: {}".format(mode_names.get(current_mode, "Unknown"))
            pl.osd_img.draw_string(20, 40, mode_txt, color=(0, 255, 0, 255))

            yolo_det.draw_result(pl, dets)
            pl.show_image()
            gc.collect()

    yolo_det.deinit()
    pl.destroy()
