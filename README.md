#代码功能
*1.识别绿灯并在图中完成标注绿灯中心的相对坐标
*2.通过uart通信发送状态（是否识别绿灯）以及中心相对坐标
*3.通过uart接受完成循环终止
*4.图片没100帧一存，帧率实时保存（均在data中）

#对飞控要求
yaw>0时要向右转向；pitch>0要低头
1.yaw_rad=34 pitch_rad=25.5 需要右转低头
EE 01 6C E7 17 3F 8C E3 E3 3E FF
2.yaw_rad=-34 pitch_rad=-25.5 左转抬头
EE 01 6C E7 17 BF 8C E3 E3 BE FF
3.yaw_rad=-34 pitch_rad=25.5 左转低头
EE 01 6C E7 17 BF 8C E3 E3 3E FF
4.yaw_rad=34 pitch_rad=-25.5 右转抬头
EE 01 6C E7 17 3F 8C E3 E3 BE FF
5.yaw_rad=34 pitch_rad=0 右转
EE 01 6C E7 17 3F 00 00 00 00 FF
6.yaw_rad=0 pitch_rad=25.5 低头
EE 01 00 00 00 00 8C E3 E3 3E FF
7.yaw_rad=-34 pitch_rad=0 左转
EE 01 6C E7 17 BF 00 00 00 00 FF
8.yaw_rad=0 pitch_rad=25.5 抬头
EE 01 00 00 00 00 8C E3 E3 BE FF

现在我需要 收到yaw>0 pitch>0时飞机协调转弯 右转低头：1.yaw_rad=34 pitch_rad=25.5 需要右转低头
uart2收到EE 01 6C E7 17 3F 8C E3 E3 3E FF
最终目标是让yaw_rad=0;pitch_rad=0
