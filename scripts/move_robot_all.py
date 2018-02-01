#!/usr/bin/env python
import time
import subprocess
import signal



subprocess.Popen(['sudo', 'chmod','777','/dev/ttyUSB0'])

proc1 = subprocess.Popen(['python', 'del_param.py'])
time.sleep(2)

proc2 = subprocess.Popen(['rosrun', 'sh_ctrl', 'serial_receiver'])
time.sleep(2)

proc3 = subprocess.Popen(['rosbag', 'record', '-o', 'trajectory', '/tool0_position'])
time.sleep(2)

proc4 = subprocess.Popen(['rosrun', 'sh_ctrl', 'robot_rot.py'])
time.sleep(2)

proc5 = subprocess.Popen(['rosrun', 'sh_ctrl', 'robot_trans.py'])
time.sleep(2)

proc6 = subprocess.Popen(['rosrun', 'sh_ctrl', 'teleop_BCI.py'])

s = input('input 1 to end')
if s == '1':
    proc1.kill()
    proc2.kill()
    proc3.kill()
    proc4.kill()
    proc5.kill()

