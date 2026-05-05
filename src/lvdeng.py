import sensor, image, time,pyb,os,math
from pyb import UART
import ustruct,struct

#通信协议定义
RX_HEAD=0xCC#接收
RX_END=0xDD
RX_LEN=16
TX_HEAD=0xEE#发送
TX_END=0xFF

#FOV标定参数 (根据摄像头规格和分辨率调整)
FOV_X_DEG=68.0 #水平视场角
FOV_Y_DEG=51.0 #垂直视场角

#常量定义
rx_buf = bytearray()
condition = 0
roi = None              # 当前ROI
lost_count = 0          # 丢失计数
MAX_LOST = 4            # 丢失多少帧后恢复全图搜索
frame_count=0           #帧率计数
save_count=0            #照片计数
clock = time.clock()    # 追踪帧率  

#变量定义
yaw_rad = 0.0
pitch_rad = 0.0
x_ral=0.0
y_ral=0.0

#标志位定义
running = False  
last_switch=0

#识别参数L:亮度值范围 A:绿-红色彩范围 B:蓝-黄色彩范围
green_threshold   = (   92, 100, -87, 11, -20, 14)
#green_threshold = (90, 100, -10, 10, -10, 10)  # 白光阈值

#调试开关
DEBUG=True#False
# 初始化SD卡
sd = pyb.SDCard()
os.mount(sd, '/sd')

#初始化摄像头
try: 
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)  # 修改分辨率
    sensor.skip_frames(time=2000)
    IMAGE_W = sensor.width()  # 动态获取图像宽度，适应不同分辨率
    IMAGE_H = sensor.height()
    CENTER_X = IMAGE_W // 2
    CENTER_Y = IMAGE_H // 2
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=500)
    if DEBUG:
        print("[初始化] 摄像头初始化成功")
        print(f"[参数] 图像分辨率: {IMAGE_W} × {IMAGE_H}")
        print(f"[参数] 图像中心坐标: ({CENTER_X}, {CENTER_Y})")
        print(f"[参数] x坐标范围: [-{CENTER_X}, {CENTER_X}]")
        print(f"[参数] y坐标范围: [-{CENTER_Y}, {CENTER_Y}]")
        print(f"[参数] 水平视场角(FOV_X): {FOV_X_DEG}°")
        print(f"[参数] 垂直视场角(FOV_Y): {FOV_Y_DEG}°")
        print(f"[参数] yaw角度范围: [-{FOV_X_DEG/2:.1f}°, {FOV_X_DEG/2:.1f}°]")
        print(f"[参数] pitch角度范围: [-{FOV_Y_DEG/2:.1f}°, {FOV_Y_DEG/2:.1f}°]")
except Exception as e:
    print(f"[初始化]摄像头初始化失败：{e}")

#uart初始化
uart=UART(3,115200,timeout_char=200)

#函数定义
def find_green_light(img):#找绿色光源
    global roi
    if roi:
        blobs = img.find_blobs([green_threshold],roi=roi, merge=True)
    else:
        blobs=img.find_blobs([green_threshold],merge=True)
    return max(blobs, key=lambda b: b.area()) if blobs else None

def pix_to_angle(x,y):#像素坐标转角度
    dx = x - CENTER_X#以图像中心为原点，计算偏移
    dy = y - CENTER_Y
    yaw_deg=dx*(FOV_X_DEG/IMAGE_W)#根据水平视场角和图像宽度计算偏航角
    pitch_deg=dy*(FOV_Y_DEG/IMAGE_H)#根据垂直视场角和图像高度计算俯仰角
    yaw_rad=math.radians(yaw_deg)
    pitch_rad=math.radians(pitch_deg)
    return yaw_rad, pitch_rad

def uart_send(a,yaw_rad,pitch_rad):#uart 发送
    global uart;
    date=ustruct.pack("<BBffB",
                 TX_HEAD,
                 int(a),
                 float(yaw_rad),
                 float(pitch_rad),
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

#主程序入口
#创建日志文件
log_id = 0
while f"fps_{log_id}.txt" in os.listdir("/sd/data"):
    log_id += 1
f = open(f"/sd/data/fps_{log_id}.txt", "w")

if DEBUG: print("[系统] 开始主循环...")

while True:
    #1.检查UART数据
    receive=uart_read()
    if receive is not None and len(receive)==RX_LEN:
        try:
            parsed=struct.unpack('16B',receive)
            #验证帧头和帧尾
            if parsed[0]==RX_HEAD and parsed[15]==RX_END:
                last_switch=parsed[1]
                if DEBUG:print(f"[解析]last_switch={last_switch}")
                #根据last_switch更新状态
                if last_switch==1:#开始识别
                    if not running:
                        running=True
                        if DEBUG: print("[状态] 切换到运行状态")
                elif  last_switch==0:#不识别
                        if running:
                            running=False
                            if DEBUG:print("[状态]切换到停止状态")
                elif last_switch==2:#退出程序
                        if DEBUG:print("[状态]接收到结束命令")
                        break
        except struct.error:
            if DEBUG:print("[错误]失败")
            uart_send(2,0,0)
            continue
    #2.持续拍照
    try:
        img=sensor.snapshot()
    except RuntimeError:
        if DEBUG:print("[错误]拍照失败")
        uart_send(2,0,0)
        continue
    #3.只有在运行状态进行识别处理
    if running:
        #识别绿色光源
        blob=find_green_light(img)
        #if DEBUG: print(f"[识别]找到目标：{'是'if blob else '否'}") 
        x_ral=0.0
        y_ral=0.0

        if blob:
            #更新ROI
            pad=20
            x=max(blob.x()-pad,0)
            y=max(blob.y()-pad,0)
            w=min(blob.w()+pad*2,IMAGE_W-x)
            h=min(blob.h()+pad*2,IMAGE_H-y)   
            roi=(x,y,w,h)
            lost_count=0

            #计算坐标
            x_ral=blob[5]-CENTER_X
            y_ral=blob[6]-CENTER_Y
            #计算角度
            yaw_rad, pitch_rad = pix_to_angle(blob[5], blob[6])
            #发送识别结果
            uart_send(1,yaw_rad,pitch_rad)
            if DEBUG: 
                print(f"[识别] 目标坐标: x={x_ral:.1f}, y={y_ral:.1f}")
                print(f"[识别] 角度(弧度): yaw={yaw_rad:.4f}, pitch={pitch_rad:.4f}")
                print(f"[识别] 角度(度数): yaw={math.degrees(yaw_rad):.2f}°, pitch={math.degrees(pitch_rad):.2f}°")
                #在图像上标记
            img.draw_rectangle(blob[0:4],color=(255,255,255))
            img.draw_cross(blob[5],blob[6],color=(255,255,255))

        else:
            lost_count+=1
            if lost_count>MAX_LOST:
                roi=None
            uart_send(0,0,0)
        #显示信息
        img.draw_string(5,20,f"状态：{'运行'if running else '停止'}",color=(255,255,255),scale=1.0)
        img.draw_string(5,30,f"x:{x_ral:.0f}",color=(255,255,255),scale=1.0)
        img.draw_string(5,40,f"y:{y_ral:.0f}",color=(255,255,255),scale=1.0)
        # 在图像上显示角度信息
        img.draw_string(80, 25, f"Y:{math.degrees(yaw_rad):+.1f}°", color=(255,255,255))
        img.draw_string(80, 35, f"P:{math.degrees(pitch_rad):+.1f}°", color=(255,255,255))

        #保存图片
        if frame_count%100==0 and save_count<1000:
            save_path=f"/sd/data/picture/frame_{save_count}.jpg"
            img.save(save_path,quality=90)
            save_count+=1 
    else:#非运行状态
       pass
    #4.记录帧率
    current_fps=clock.fps()
    f.write(f"{current_fps:.2f}\n")
    frame_count+=1

    #每100帧刷新文件缓冲区
    if frame_count%100==0:
        f.flush()
        os.sync()
#清理工作
f.close()
os.umount('/sd')
if DEBUG: print("[系统] 程序结束")