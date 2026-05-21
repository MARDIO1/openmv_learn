import sensor, image, time, math, mjpeg, os
from pyb import UART
import ustruct

# ==============================================
# 通信协议定义
# ==============================================
TX_HEAD = 0xEE
TX_END  = 0xFF

# ==============================================
# FOV 标定参数
# ==============================================
FOV_X_DEG = 68.0
FOV_Y_DEG = 51.0

# ==============================================
# 识别参数
# ==============================================
green_threshold = (80, 100, -100, 11, -20, 20)
MAX_LOST = 4

# ==============================================
# 内录参数
# ==============================================
REC_ENABLE = True

# 建议 2~5fps
REC_FPS = 35
REC_INTERVAL_MS = int(1000 / REC_FPS)

# 单次录制最长时间
REC_DURATION_MS = 15000

# 目标丢失超过多久停止录制
LOST_STOP_MS = 2000

# JPEG 质量
REC_QUALITY = 20

# 自动文件名
REC_PREFIX = "rec_"
REC_EXT = ".mjpeg"

# ==============================================
# 状态变量
# ==============================================
roi = None
lost_count = 0

last_yaw_rad   = 0.0
last_pitch_rad = 0.0
last_frame_time_us = 0

# 录制相关
recording = False
stream = None
frame_cnt = 0
record_start_ms = 0
last_record_ms = 0
lost_since_ms = 0
mjpeg_quality_supported = True
current_record_name = ""

# 0x02 是否已发
sent_0x02 = False

# ==============================================
# 摄像头初始化
# ==============================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=450)
sensor.skip_frames(time=500)

IMAGE_W  = sensor.width()
IMAGE_H  = sensor.height()
CENTER_X = IMAGE_W // 2
CENTER_Y = IMAGE_H // 2

# ==============================================
# UART 初始化
# ==============================================
uart = UART(3, 115200, timeout_char=200)

# ==============================================
# 函数定义
# ==============================================
def find_green_light(img):
    global roi, lost_count

    blobs = []

    # 核心修复：先ROI搜索，找不到自动全局搜索
    if roi:
        # 第一步：优先在ROI内搜索（速度快）
        blobs = img.find_blobs([green_threshold], roi=roi, merge=False, pixels_threshold=10, area_threshold=10)

        # 第二步：ROI内没找到，立即全局搜索
        if not blobs:
            blobs = img.find_blobs([green_threshold], merge=False, pixels_threshold=10, area_threshold=10)
    else:
        # 没有ROI时直接全局搜索
        blobs = img.find_blobs([green_threshold], merge=False, pixels_threshold=10, area_threshold=10)

    return max(blobs, key=lambda b: b.area()) if blobs else None

def pix_to_angle(x, y):
    dx = x - CENTER_X
    dy = y - CENTER_Y

    yaw_deg   = dx * (FOV_X_DEG / IMAGE_W)
    pitch_deg = dy * (FOV_Y_DEG / IMAGE_H)

    return math.radians(yaw_deg), math.radians(pitch_deg)


def uart_send(switch, yaw_radps, pitch_radps):
    data = ustruct.pack(
        "<BBffB",
        TX_HEAD,
        int(switch),
        float(yaw_radps),
        float(pitch_radps),
        TX_END
    )
    uart.write(data)


def get_next_mjpeg_name():
    idx = 0

    while True:
        name = "%s%03d%s" % (REC_PREFIX, idx, REC_EXT)

        try:
            os.stat(name)
            idx += 1
        except OSError:
            return name


def start_record():
    global recording, stream, frame_cnt
    global record_start_ms, last_record_ms
    global current_record_name

    if not REC_ENABLE:
        return False

    if recording:
        return True

    try:
        file_name = get_next_mjpeg_name()

        stream = mjpeg.Mjpeg(file_name)
        current_record_name = file_name

        recording = True
        frame_cnt = 0

        now_ms = time.ticks_ms()
        record_start_ms = now_ms
        # 只有识别到绿灯后，在 blob 分支里按 REC_FPS 写入
        last_record_ms = now_ms

        print("Record start:", current_record_name)

        return True

    except Exception as e:
        print("Record start error:", e)
        stream = None
        recording = False
        current_record_name = ""
        return False


def stop_record():
    global recording, stream
    global current_record_name

    if not recording:
        return

    try:
        if stream:
            stream.close()
    except Exception as e:
        print("Record close error:", e)

    print("Record stop:", current_record_name)

    stream = None
    recording = False
    current_record_name = ""


def mjpeg_add_frame_safe(img):
    global stream, mjpeg_quality_supported

    if not recording or stream is None:
        return False

    try:
        if mjpeg_quality_supported:
            try:
                stream.add_frame(img, quality=REC_QUALITY)
            except TypeError:
                mjpeg_quality_supported = False
                stream.add_frame(img)
        else:
            stream.add_frame(img)

        return True

    except Exception as e:
        print("Add frame error:", e)
        stop_record()
        return False


def record_frame_if_needed(img):
    global last_record_ms, frame_cnt

    if not recording:
        return

    now_ms = time.ticks_ms()

    if time.ticks_diff(now_ms, last_record_ms) >= REC_INTERVAL_MS:
        if mjpeg_add_frame_safe(img):
            frame_cnt += 1

        last_record_ms = now_ms


def update_roi(blob):
    global roi

    pad = 20

    x = max(blob.x() - pad, 0)
    y = max(blob.y() - pad, 0)
    w = min(blob.w() + pad * 2, IMAGE_W - x)
    h = min(blob.h() + pad * 2, IMAGE_H - y)

    roi = (x, y, w, h)


def check_record_timeout():
    global sent_0x02

    if not recording:
        return

    now_ms = time.ticks_ms()

    if time.ticks_diff(now_ms, record_start_ms) >= REC_DURATION_MS:
        stop_record()
        sent_0x02 = False


def check_lost_timeout():
    global sent_0x02

    if not recording:
        return

    if lost_since_ms == 0:
        return

    now_ms = time.ticks_ms()

    if time.ticks_diff(now_ms, lost_since_ms) >= LOST_STOP_MS:
        stop_record()
        sent_0x02 = False


# ==============================================
# 主循环
# ==============================================
clock = time.clock()

while True:
    clock.tick()

    # 1. 拍照
    try:
        img = sensor.snapshot()
    except RuntimeError:
        uart_send(0, 0.0, 0.0)
        continue

    now_us = time.ticks_us()
    now_ms = time.ticks_ms()

    # 2. 识别绿灯
    blob = find_green_light(img)

    if blob:
        lost_count = 0
        lost_since_ms = 0

        update_roi(blob)

        yaw_rad, pitch_rad = pix_to_angle(blob.cx(), blob.cy())

        yaw_body_radps = 0.0
        pitch_body_radps = 0.0

        if last_frame_time_us > 0:
            dt_s = time.ticks_diff(now_us, last_frame_time_us) / 1000000.0

            if 0.001 < dt_s < 1.0:
                yaw_body_radps   = (yaw_rad   - last_yaw_rad)   / dt_s
                pitch_body_radps = (pitch_rad - last_pitch_rad) / dt_s

        last_yaw_rad = yaw_rad
        last_pitch_rad = pitch_rad
        last_frame_time_us = now_us

        # ======================================
        # 发现绿灯后才启动录像、写入录像
        # ======================================
        if not recording:
            if not sent_0x02:
                uart_send(2, yaw_body_radps, pitch_body_radps)
                sent_0x02 = True

            start_record()

        else:
            uart_send(1, yaw_body_radps, pitch_body_radps)

        # 重点：
        # 只有在 blob 存在，也就是确实识别到绿灯时，才写入视频帧
        record_frame_if_needed(img)

        check_record_timeout()

        # ======================================
        # IDE 显示
        # ======================================
        # ======================================
        # IDE 显示
        # ======================================
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0))

        img.draw_string(
            blob.x() + 2,
            max(blob.y() - 10, 0),
            "yaw:" + str(round(math.degrees(yaw_rad), 1)) + "d",
            color=(255, 0, 0)
        )

        img.draw_string(
            0,
            0,
            ("REC " if recording else "") + "FPS:" + str(round(clock.fps(), 1)),
            color=(255, 0, 0) if recording else (0, 255, 255)
        )
