#!/usr/bin/python3
import Jetson.GPIO as GPIO
import time
import subprocess
import os


# 设置 GPIO 模式
GPIO.setmode(GPIO.BOARD)  # 使用引脚编号方式

# 设置 GPIO 12 为输入模式
input_pin = 7
output_pin = 40
GPIO.setup(output_pin, GPIO.OUT)
GPIO.setup(input_pin, GPIO.IN)

# 保存进程信息的变量
process = None
GPIO.output(output_pin, GPIO.LOW)
try:
    command = "xrandr --fb 1900x1240"
    subprocess.run(command, shell=True)
    GPIO.output(output_pin, GPIO.LOW)
    while True:
        # 检测引脚电平状态
        if GPIO.input(input_pin) == GPIO.HIGH:
            print("检测到高电平，执行另一个程序")
            # 执行另一个程序（例如运行一个脚本）并避免阻塞主程序
            if process is not None and process.poll() is None:  # 检查进程是否仍在运行
                time.sleep(1)  # 每隔1秒检测一次
                continue
            command = "echo '0000' | sudo -S chmod 777 /dev/ttyTHS1"
            subprocess.run(command, shell=True)
            folder_path = "/home/user/code/"  # 替换为你要进入的目录路径
            os.chdir(folder_path)
            
            #process = subprocess.Popen(
            #    ["xterm", "-e", "/usr/bin/python3", "/home/user/code/jetson_nano_main.py"]
            #)
            process = subprocess.Popen(
                ["xterm", "-e", "/usr/bin/python3", "/home/user/code/jetson_nano_main_final.py"]
           )

        else:
            if process is not None and process.poll() is None:  # 检查进程是否仍在运行
                print("终止先前运行的程序")
                process.terminate()  # 可以用 kill() 强制关闭
                process.wait()       # 等待进程完全结束
            GPIO.output(output_pin, GPIO.LOW)
            print("低电平，熄燈")

        time.sleep(1)  # 每隔1秒检测一次

except KeyboardInterrupt:
    # 清理 GPIO 设置
    GPIO.cleanup()