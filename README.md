# Parallel-robotic-arm-control-codes-in-stm32c8t6-based-on-HAL #
在main.c里面使用串口通信，参考上位机程序arm_control.py
允许输入空间三维坐标(需要offset根据实际)，采用s性插值使其运动平滑。二值控制夹取。
物理参数，角度初值等机械臂校准可在motor更改。
