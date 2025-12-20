import sensor, image, time,pyb,os
from pyb import UART
import ustruct
# 初始化SD卡
sd = pyb.SDCard()

# 挂载SD卡到/sd目录
os.mount(sd, '/sd')
green_threshold   = (   50,   100,  -128,   -30,   -128,   127)
#设置绿色的阈值，括号里面的数值分别是L A B 的最大值和最小值（minL, maxL, minA,
# maxA, minB, maxB），LAB的值在图像左侧三个坐标图中选取。如果是灰度图，则只需
#设置（min, max）两个数字即可。
f=open("/sd/data/fps.txt","w")
sensor.reset() # 初始化摄像头
sensor.set_pixformat(sensor.RGB565) # 格式为 RGB565.
sensor.set_framesize(sensor.QQVGA) # 使用 QQVGA 速度快一些
sensor.skip_frames(time = 2000) # 跳过2000ms，使新设置生效,并自动调节白平衡
sensor.set_auto_gain(False) # 关闭自动自动增益。默认开启的，在颜色识别中，一定要关闭白平衡。
sensor.set_auto_whitebal(False)
#关闭白平衡。白平衡是默认开启的，在颜色识别中，一定要关闭白平衡。
uart = UART(3, 115200,timeout_char=200)
frame_count=0#帧率计数
save_count=0#照片计数
clock = time.clock() # 追踪帧率
x_ral=0.0
y_ral=0.0
a=0#数据初始化
center_x=sensor.width()//2
center_y=sensor.height()//2
def uart_send(a,x,y):#uart 发送
    global uart;
    date=ustruct.pack("<bbffb",
                 0xEE,
                 int(a),
                 float(x),
                 float(y),
                 0xFF)
    uart.write(date)


while(frame_count<=5000):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # 从感光芯片获得一张图像
    frame_count+=1
    if uart.any():
        break
   




    blobs = img.find_blobs([green_threshold], area_threshold=100, merge=True)
    #find_blobs(thresholds, invert=False, roi=Auto),thresholds为颜色阈值，
    #是一个元组，需要用括号［ ］括起来。invert=1,反转颜色阈值，invert=False默认
    #不反转。roi设置颜色识别的视野区域，roi是一个元组， roi = (x, y, w, h)，代表
    #从左上顶点(x,y)开始的宽为w高为h的矩形区域，roi不设置的话默认为整个图像视野。
    #这个函数返回一个列表，[0]代表识别到的目标颜色区域左上顶点的x坐标，［1］代表
    #左上顶点y坐标，［2］代表目标区域的宽，［3］代表目标区域的高，［4］代表目标
    #区域像素点的个数，［5］代表目标区域的中心点x坐标，［6］代表目标区域中心点y坐标，
    #［7］代表目标颜色区域的旋转角度（是弧度值，浮点型，列表其他元素是整型），
    #［8］代表与此目标区域交叉的目标个数，［9］代表颜色的编号（它可以用来分辨这个
    #区域是用哪个颜色阈值threshold识别出来的）。
    count=0
    a=0
    x_ral=0.0
    y_ral=0.0
    if blobs:

    #如果找到了目标颜色

        for b in blobs:
        #迭代找到的目标颜色区域
            # Draw a rect around the blob.
            img.draw_rectangle(b[0:4],color=(0,255,0)) # rect
            #用矩形标记出目标颜色区域
            img.draw_cross(b[5], b[6],color=(255,0,0))# cx, cy
            #在目标颜色区域的中心画十字形标记
            x_ral=b[5]-center_x # x_rel是x中心坐标（中心值）
            y_ral=b[6]-center_y
            count+=1
            a=1
    img.draw_string(5,15,f"NMU:{count:.0f}",color=(255,255,255), scale=1.0)
    img.draw_string(5,25,f"x:{x_ral:.0f}",color=(255,255,255), scale=1.0)
    img.draw_string(5,35,f"y:{y_ral:.0f}",color=(255,255,255), scale=1.0)
            #获取图像
    if frame_count%100==0 and save_count<1000:
    #图片存储限制
        save_path = f"/sd/data/picture/frame_{save_count}.jpg" # SD卡路径+jpg格式+唯一命名
        img.save(save_path, quality=90)  # quality可选，默认90
        save_count += 1  # 保存计数器+1，避免覆盖
    uart_send(a,x_ral,y_ral)
    #如果断开电脑，帧率会增加
    f.write(str(clock.fps())+"\n")
f.close()
os.umount('/sd')
