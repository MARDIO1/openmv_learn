# OpenMV绿光识别程序
import sensor, image, time
from pyb import UART

# 初始化摄像头
sensor.reset()#一键初始化，方便的
sensor.set_pixformat(sensor.RGB565)#设置像素格式为RGB565
sensor.set_framesize(sensor.QVGA)#设置分辨率
sensor.skip_frames(time=2000)#相当于delay初始化等稳定
sensor.set_auto_gain(False)#关闭自动增益
sensor.set_auto_whitebal(False)#自动白平衡功能

# 初始化串口
uart = UART(3, 115200)

# 绿色阈值 (LAB颜色空间)参数依次是 最小亮度 最大亮度 最小a 最大a 最小b 最大b，其中av
green_threshold = (50, 100, -128, -30, -128, 127)

# 帧率统计，初始化一个时钟对象，待会会调用tick()方法用来计算FPS
clock = time.clock()

def find_green_light(img):#找绿色光源
    #find_blobs：返回一个 色块 对象 
    #green_threshold 颜色条件 area_threshold最小面积条件 merge色块合并条件
    blobs = img.find_blobs([green_threshold], area_threshold=100, merge=True)
    #再次筛选，过滤面积小的和面积太大的色块 哇塞，代码压缩这么厉害的吗
    valid_blobs = [b for b in blobs if 100 <= b.area() <= 10000]
    #返回最大的色块
    return max(valid_blobs, key=lambda b: b.area()) if valid_blobs else None

def draw_target(img, blob):#绘制目标信息
    if not blob: return
    #用于在图像中绘制矩形框
    img.draw_rectangle(blob.rect(), color=(0, 255, 0))
    #用于在图像中绘制十字架 颜色是红色，刚好是对比色，合理的
    img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0), size=10)
    #blob.cx()+5使其稍微偏移色块的中心x坐标，为什么这么做？ 哦哦这个是字啊，唐完了
    #f"X:{blob.cx():3d} Y:{blob.cy():3d}": 使用格式化字符串来显示色块的中心坐标。blob.cx():3d 表示中心x坐标，右对齐且至少占3位；blob.cy():3d 表示中心y坐标，右对齐且至少占3位。
    img.draw_string(blob.cx()+5, blob.cy()-20, f"X:{blob.cx():3d} Y:{blob.cy():3d}", color=(255,255,255), scale=1.2)

def send_data(blob):#发送数据
    if blob:
        uart.write(f"X:{blob.cx():3d},Y:{blob.cy():3d},A:{blob.area():4d}\n")

def main():
    print("绿光识别程序启动")
    while True:
        clock.tick()#计算上一次调用的时间间隔
        img.draw_string(5, 5, f"FPS:{clock.fps():.1f}", color=(255,255,255), scale=1.5)
        #获取图像
        img = sensor.snapshot()
        #获取绿色光源，准备画出来
        target = find_green_light(img)
        draw_target(img, target)
        send_data(target)
        
        img.draw_string(5, 25, "Target Found" if target else "Searching", color=(255,255,255), scale=1.2)

if __name__ == "__main__":
    main()
