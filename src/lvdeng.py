import sensor, image, time,pyb,os
from pyb import UART
import ustruct,struct
#调试开关
DEBUG=True#False

# 初始化SD卡
sd = pyb.SDCard()

# 挂载SD卡到/sd目录
os.mount(sd, '/sd')
green_threshold   = (   83, 100, -32, -18, -3, -20)
uart = UART(3, 115200,timeout_char=200)
frame_count=0#帧率计数
save_count=0#照片计数
clock = time.clock() # 追踪帧率
x_ral=0.0
y_ral=0.0
state=0
num=0
#通信协议定义
RX_HEAD=0xCC#接收
RX_END=0xDD
RX_LEN=16
TX_HEAD=0xEE#发送
TX_END=0xFF
rx_buf = bytearray()
condition = 0
roi = None              # 当前ROI
lost_count = 0          # 丢失计数
MAX_LOST = 4            # 丢失多少帧后恢复全图搜索

def find_green_light(img):#找绿色光源
    global roi
    if roi:
        blobs = img.find_blobs([green_threshold],roi=roi, merge=True)
    else:
        blobs=img.find_blobs([green_threshold],merge=True)
    return max(blobs, key=lambda b: b.area()) if blobs else None

def uart_send(a,x,y):#uart 发送
    global uart;
    date=ustruct.pack("<BBffB",
                 TX_HEAD,
                 int(a),
                 float(x),
                 float(y),
                 TX_END)
    uart.write(date)

def uart_read():#uart 接收
    global uart,rx_buf,condition;
    while uart.any():
            byte = uart.readchar()
            if DEBUG:print(f"[UART]接收到字节：0x{byte:02X}")
            # 等待帧头
            if condition == 0:
                if byte == RX_HEAD:
                    if DEBUG:print(f"[UART]找到帧头：0x{RX_HEAD:02X}")
                    rx_buf = bytearray([byte])
                    condition = 1

            # 正在接收一帧
            elif condition == 1:
                rx_buf.append(byte)

                if len(rx_buf) == RX_LEN:
                    condition = 0

                    if rx_buf[-1] == RX_END:
                        return rx_buf  # 成功接收一帧
                    else:
                        rx_buf = bytearray()  # 帧错误，丢弃
                        return None
    return None
log_id = 0
while f"fps_{log_id}.txt" in os.listdir("/sd/data"):
    log_id += 1

f = open(f"/sd/data/fps_{log_id}.txt", "w")

while(True):
    receive=uart_read()
    if receive is None:
        if DEBUG:print("[UART]未接收到数据")
        continue
    if len(receive)!=RX_LEN:
        if DEBUG:print(f"[UART]数据长度错误：期望{RX_LEN},实际{len(receive)}")
        continue
    if DEBUG:print(f"[UART]接收完整帧：{receive.hex()}")
    fmt='16B'
    try:
        parsed=struct.unpack(fmt,receive)
    except struct.error:
        continue
    result = {
            'frame_head': parsed[0],
            'last_switch': parsed[1],
            'blackbox_data': parsed[2:14],  # 索引2到13
            #暂时不进行crc'crc':parsed[14],
            'frame_tail': parsed[15]
        }
    if DEBUG:print(f"[解析]last_switch={result['last_switch']},帧尾=0x{result['frame_tail']}")
    
    #状态机切换逻辑
    frame_head=result["frame_head"]
    last_switch=result["last_switch"]
    frame_tail=result["frame_tail"]
    if DEBUG:print(f"[状态机]接收到last_switch={last_switch},当前state={state}")
    #根据last_switch切换状态机
    if last_switch!=1:
        state=0
        if DEBUG:print(f"[状态机]切换到停止状态：state={state}")
        uart_send(state,0,0)
        continue
    elif last_switch==3:
        if DEBUG:print("[状态机]接收到结束命令，退出程序")
        break
        
#摄像头初始化
    if (num==0):
        try:
            if DEBUG:print("[摄像头]开始初始化...")
            sensor.reset() # 初始化摄像头
            sensor.set_pixformat(sensor.RGB565) # 格式为 RGB565.
            sensor.set_framesize(sensor.QVGA) # 使用 QQVGA 速度快一些
            sensor.skip_frames(time = 2000) # 跳过2000ms，使新设置生效,并自动调节白平衡
            sensor.set_auto_gain(False) # 关闭自动自动增益。默认开启的，在颜色识别中，一定要关闭白平衡。
            sensor.set_auto_whitebal(False)
            sensor.set_auto_exposure(False,exposure_us=500)
        except OSError as e:
            print(f"初始化失败 (OSError): {e}")
            state=2
            if DEBUG:print(f"[摄像头]初始化失败，state={state}")
            uart_send(state,0,0)
            continue
        else:
            num=1
            state=1
            if DEBUG:print(f"[摄像头]初始化成功，state={state}")
            center_x=sensor.width()//2
            center_y=sensor.height()//2

    blackbox_data=result["blackbox_data"]
    clock.tick() # Track elapsed milliseconds between snapshots().
   
    try:
        img = sensor.snapshot()
    except RuntimeError:
        state=2
        if DEBUG:print("[错误]拍照失败")
        uart_send(state,0,0)
        continue
    #图像识别
    blob =find_green_light(img)
    if DEBUG:print(f"[识别]找到目标：{'是'if blob else'否'}")

    count=0
    a=0
    x_ral=0.0
    y_ral=0.0
    if blob:
        pad = 20  # ROI边界扩展（很关键）

        x = max(blob.x() - pad, 0)
        y = max(blob.y() - pad, 0)
        w = min(blob.w() + pad*2, sensor.width()-x)
        h = min(blob.h() + pad*2, sensor.height()-y)

        roi = (x, y, w, h)
        lost_count = 0

        #如果找到了目标颜色
        # Draw a rect around the blob.
        img.draw_rectangle(blob[0:4],color=255)
        #用矩形标记出目标颜色区域
        img.draw_cross(blob[5], blob[6],color=255)# cx, cy
        #在目标颜色区域的中心画十字形标记
        x_ral=blob[5]-center_x # x_rel是x中心坐标（中心值）
        y_ral=blob[6]-center_y
        if DEBUG:print(f"[识别]目标坐标：x={x_ral:.1f},y={y_ral:.1f}")
        count+=1
    else:
        lost_count+=1
        if lost_count>MAX_LOST:
            roi=None
    img.draw_string(5,15,f"NMU:{count:.0f}",color=255, scale=1.0)
    img.draw_string(5,25,f"x:{x_ral:.0f}",color=255, scale=1.0)
    img.draw_string(5,35,f"y:{y_ral:.0f}",color=255, scale=1.0)
            #获取图像
    if frame_count%100==0 and save_count<1000:

        save_path = f"/sd/data/picture/frame_{save_count}.jpg" # SD卡路径+jpg格式+唯一命名
        img.save(save_path, quality=90)  # quality可选，默认90
        save_count += 1  # 保存计数器+1，避免覆盖
    uart_send(state,x_ral,y_ral)
    if DEBUG:print(f"[发送]state={state},x={x_ral:.1f},y={y_ral:.1f}")
    #如果断开电脑，帧率会增加
    current_fps = clock.fps()
               # 格式：帧率 + 空格 + blackbox_data
    f.write(f"{current_fps:.2f}\n")
    frame_count += 1

               # 每100帧刷新文件缓冲区
    if frame_count % 100 == 0:
       f.flush()
       os.sync()
f.close()
os.umount('/sd')
