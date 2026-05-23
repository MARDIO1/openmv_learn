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
# 绿色色块阈值
# ==============================================
green_threshold = (50, 100, -100, -10, -20, 20)

# ==============================================
# 摄像头参数
# ==============================================
EXPOSURE_US = 1500

# ==============================================
# 唤醒保持时间
# 识别到色块后保持录像/唤醒 4 秒
# 4秒内再次识别到色块会重新刷新倒计时
# ==============================================
WAKE_HOLD_MS = 4000

# ==============================================
# 内录参数
# ==============================================
REC_ENABLE = True

REC_FPS = 25
REC_INTERVAL_MS = int(1000 / REC_FPS)

# MJPEG质量，越高越清晰但越卡
REC_QUALITY = 30

REC_PREFIX = "/sd/rec_"
REC_EXT = ".mjpeg"

# ==============================================
# ROI 参数
# ==============================================
USE_ROI = True
ROI_PAD = 30
MAX_LOST_FOR_FULL_SEARCH = 4

# ==============================================
# 状态变量
# ==============================================
roi = None
lost_count = 0

last_yaw_rad = 0.0
last_pitch_rad = 0.0
last_frame_time_us = 0

last_x = 0
last_y = 0
last_yaw_d = 0.0
last_pitch_d = 0.0
last_yaw_dps = 0.0
last_pitch_dps = 0.0

awake = False
wake_until_ms = 0

recording = False
stream = None
frame_cnt = 0
record_start_ms = 0
last_record_ms = 0
mjpeg_quality_supported = True
current_record_name = ""

# 用于判断是不是新唤醒
was_awake_last_loop = False


# ==============================================
# SD卡检测
# ==============================================
def sd_available():
    try:
        os.listdir("/sd")
        return True
    except Exception as e:
        print("SD card not available:", e)
        return False


if not sd_available():
    print("Warning: SD card not found.")
    print("Fallback to internal path. Not recommended.")
    REC_PREFIX = "rec_"


# ==============================================
# 摄像头初始化
# ==============================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)

# 如果 QVGA 太卡，改成 sensor.QQVGA
sensor.set_framesize(sensor.QVGA)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

try:
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
except TypeError:
    try:
        sensor.set_auto_exposure(False, EXPOSURE_US)
    except TypeError:
        sensor.set_auto_exposure(False)

try:
    sensor.skip_frames(time=500)
except TypeError:
    sensor.skip_frames()

IMAGE_W = sensor.width()
IMAGE_H = sensor.height()
CENTER_X = IMAGE_W // 2
CENTER_Y = IMAGE_H // 2

print("Image size:", IMAGE_W, IMAGE_H)


# ==============================================
# UART 初始化
# ==============================================
try:
    uart = UART(3, 115200, timeout_char=200)
except TypeError:
    uart = UART(3, 115200)
    try:
        uart.init(115200, bits=8, parity=None, stop=1, timeout_char=200)
    except TypeError:
        uart.init(115200)

print("UART init OK")


# ==============================================
# 函数定义
# ==============================================
def uart_send(switch, yaw_radps, pitch_radps):
    try:
        data = ustruct.pack(
            "<BBffB",
            TX_HEAD,
            int(switch),
            float(yaw_radps),
            float(pitch_radps),
            TX_END
        )
        uart.write(data)
    except Exception as e:
        print("UART send error:", e)


def pix_to_angle(x, y):
    dx = x - CENTER_X
    dy = y - CENTER_Y

    yaw_deg = dx * (FOV_X_DEG / IMAGE_W)
    pitch_deg = dy * (FOV_Y_DEG / IMAGE_H)

    return math.radians(yaw_deg), math.radians(pitch_deg)


def update_roi(blob):
    global roi

    x = max(blob.x() - ROI_PAD, 0)
    y = max(blob.y() - ROI_PAD, 0)
    w = min(blob.w() + ROI_PAD * 2, IMAGE_W - x)
    h = min(blob.h() + ROI_PAD * 2, IMAGE_H - y)

    roi = (x, y, w, h)


def find_green_light(img):
    global roi

    blobs = []

    if USE_ROI and roi:
        blobs = img.find_blobs(
            [green_threshold],
            roi=roi,
            pixels_threshold=10,
            area_threshold=10,
            merge=False
        )

        if blobs:
            return max(blobs, key=lambda b: b.area())

    blobs = img.find_blobs(
        [green_threshold],
        pixels_threshold=10,
        area_threshold=10,
        merge=False
    )

    if blobs:
        return max(blobs, key=lambda b: b.area())

    return None


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
        last_record_ms = now_ms

        print("================================")
        print("Record start:", current_record_name)
        print("REC_FPS:", REC_FPS)
        print("REC_QUALITY:", REC_QUALITY)
        print("Wake hold ms:", WAKE_HOLD_MS)
        print("================================")

        return True

    except Exception as e:
        print("Record start error:", e)
        stream = None
        recording = False
        current_record_name = ""
        return False


def stop_record():
    global recording, stream, current_record_name

    if not recording:
        return

    try:
        if stream:
            stream.close()
    except Exception as e:
        print("Record close error:", e)

    print("================================")
    print("Record stop:", current_record_name)
    print("Total frames:", frame_cnt)
    print("================================")

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
                try:
                    stream.add_frame(img, REC_QUALITY)
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


def record_frame_force(img):
    global last_record_ms, frame_cnt

    if not recording:
        return

    if mjpeg_add_frame_safe(img):
        frame_cnt += 1

    last_record_ms = time.ticks_ms()


def draw_tracking_ui(img, has_target, fps):
    # 基础状态
    if has_target:
        img.draw_string(0, 0, "TARGET", color=(0, 255, 0))
    elif awake:
        img.draw_string(0, 0, "WAKE HOLD", color=(255, 180, 0))
    else:
        img.draw_string(0, 0, "SLEEP", color=(0, 255, 255))

    # 必须显示的数据
    img.draw_string(0, 12, "x:%d y:%d" % (last_x, last_y), color=(0, 255, 255))
    img.draw_string(0, 24, "yaw_d:%.1f" % last_yaw_d, color=(0, 255, 255))
    img.draw_string(0, 36, "pitch_d:%.1f" % last_pitch_d, color=(0, 255, 255))
    img.draw_string(0, 48, "yaw_dps:%.2f" % last_yaw_dps, color=(0, 255, 255))
    img.draw_string(0, 60, "pitch_dps:%.2f" % last_pitch_dps, color=(0, 255, 255))

    # 录像状态
    if recording:
        elapsed_ms = time.ticks_diff(time.ticks_ms(), record_start_ms)
        if elapsed_ms < 0:
            elapsed_ms = 0

        left_ms = time.ticks_diff(wake_until_ms, time.ticks_ms())
        if left_ms < 0:
            left_ms = 0

        img.draw_string(0, 76, "REC F:%d" % frame_cnt, color=(255, 0, 0))
        img.draw_string(0, 88, "hold:%.1fs" % (left_ms / 1000.0), color=(255, 0, 0))
        img.draw_string(0, 100, "time:%.1fs" % (elapsed_ms / 1000.0), color=(255, 0, 0))

    img.draw_string(180, 0, "FPS:%.1f" % round(fps, 1), color=(255, 0, 0) if recording else (0, 255, 255))


# ==============================================
# 主循环
# ==============================================
clock = time.clock()

print("Program start.")
print("Logic: always detect green blob.")
print("Green detected -> wake and record for 4 seconds.")
print("Repeated detection extends wake time.")

while True:
    clock.tick()

    try:
        img = sensor.snapshot()
    except RuntimeError:
        uart_send(0, 0.0, 0.0)
        continue

    now_ms = time.ticks_ms()
    now_us = time.ticks_us()

    # ==========================================
    # 一直识别绿色色块
    # ==========================================
    blob = find_green_light(img)

    has_target = False

    if blob:
        has_target = True
        lost_count = 0

        update_roi(blob)

        cx = blob.cx()
        cy = blob.cy()

        yaw_rad, pitch_rad = pix_to_angle(cx, cy)

        yaw_radps = 0.0
        pitch_radps = 0.0

        if last_frame_time_us > 0:
            dt_s = time.ticks_diff(now_us, last_frame_time_us) / 1000000.0

            if 0.001 < dt_s < 1.0:
                yaw_radps = (yaw_rad - last_yaw_rad) / dt_s
                pitch_radps = (pitch_rad - last_pitch_rad) / dt_s

        last_frame_time_us = now_us
        last_yaw_rad = yaw_rad
        last_pitch_rad = pitch_rad

        last_x = cx
        last_y = cy
        last_yaw_d = round(math.degrees(yaw_rad), 1)
        last_pitch_d = round(math.degrees(pitch_rad), 1)
        last_yaw_dps = round(yaw_radps, 2)
        last_pitch_dps = round(pitch_radps, 2)

        # 画目标框和十字
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
        img.draw_cross(cx, cy, color=(255, 0, 0))

        # 每次识别到目标，都刷新 4 秒唤醒时间
        wake_until_ms = time.ticks_add(now_ms, WAKE_HOLD_MS)

        # 如果之前不是唤醒状态，这次是新唤醒
        if not awake:
            awake = True

            # 新唤醒瞬间发送 0x02
            uart_send(2, yaw_radps, pitch_radps)

            # 开始录像
            if start_record():
                print("Wake by green blob.")
                print("x:", cx, "y:", cy)
                print("yaw_d:", last_yaw_d)
                print("pitch_d:", last_pitch_d)
        else:
            # 已经唤醒时，有目标则发送 0x01
            uart_send(1, yaw_radps, pitch_radps)

    else:
        lost_count += 1

        if lost_count > MAX_LOST_FOR_FULL_SEARCH:
            roi = None

        # 没有目标，但可能还在 4秒保持期
        if awake:
            uart_send(1, 0.0, 0.0)
        else:
            uart_send(0, 0.0, 0.0)

    # ==========================================
    # 判断唤醒是否超时
    # ==========================================
    if awake:
        if time.ticks_diff(wake_until_ms, now_ms) <= 0:
            awake = False
            wake_until_ms = 0

            uart_send(0, 0.0, 0.0)

            if recording:
                stop_record()

    # ==========================================
    # 绘制 UI
    # 注意：必须在 add_frame 之前画，录像里才有 UI
    # ==========================================
    draw_tracking_ui(img, has_target, clock.fps())

    # ==========================================
    # 录像写帧
    # 注意：UI 已经画完，所以录像包含 UI
    # ==========================================
    if recording:
        record_frame_if_needed(img)
