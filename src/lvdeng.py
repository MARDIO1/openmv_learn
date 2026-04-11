import sensor, image, time,pyb,os
from pyb import UART
import ustruct,struct
#通信协议定义
RX_HEAD=0xCC#接收
RX_END=0xDD
RX_LEN=16
TX_HEAD=0xEE#发送
TX_END=0xFF

#常量定义
x_ral=0.0
y_ral=0.0
rx_buf = bytearray()
condition = 0
roi = None              # 当前ROI
lost_count = 0          # 丢失计数
MAX_LOST = 4            # 丢失多少帧后恢复全图搜索
frame_count=0           #帧率计数
save_count=0            #照片计数
clock = time.clock()    # 追踪帧率

#标志位定义
running = False  
last_switch=0

#识别参数
green_threshold   = (   83, 100, -32, -18, -3, -20)
#green_threshold = (90, 100, -10, 10, -10, 10)  # 白光阈值

#调试开关
DEBUG=True#False
# 初始化SD卡
sd = pyb.SDCard()
os.mount(sd, '/sd')

#初始化摄像头
try:
    if DEBUG:print("[初始化]开始摄像头初始化...")
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=500)
    if DEBUG: print("[初始化] 摄像头初始化成功")
except Exception as e:
    print(f"[初始化]摄像头初始化失败：{e}")
center_x=sensor.width()//2
center_y=sensor.height()//2
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
        if DEBUG: print(f"[识别]找到目标：{'是'if blob else '否'}") 
        x_ral=0.0
        y_ral=0.0

        if blob:
            #更新ROI
            pad=20
            x=max(blob.x()-pad,0)
            y=max(blob.y()-pad,0)
            w=min(blob.w()+pad*2,sensor.width()-x)
            h=min(blob.h()+pad*2,sensor.height()-y)   
            roi=(x,y,w,h)
            lost_count=0

            #计算坐标
            x_ral=blob[5]-center_x
            y_ral=blob[6]-center_y
            if DEBUG: print(f"[识别] 目标坐标: x={x_ral:.1f}, y={y_ral:.1f}")

            #在图像上标记
            img.draw_rectangle(blob[0:4],color=255)
            img.draw_cross(blob[5],blob[6],color=255)
        else:
            lost_count+=1
            if lost_count>MAX_LOST:
                roi=None
        #显示信息
        img.draw_string(5,15,f"状态：{'运行'if running else '停止'}",color=255,scale=1.0)
        img.draw_string(5,25,f"x:{x_ral:.0f}",color=255,scale=1.0)
        img.draw_string(5,35,f"y:{y_ral:.0f}",color=255,scale=1.0)

        #发送识别结果
        uart_send(1,x_ral,y_ral)

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