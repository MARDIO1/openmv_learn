import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
clock = time.clock()
#阈值编辑器:Tools → Machine Vision → Threshold Editor
while True:
    clock.tick()
    img = sensor.snapshot()