#!/usr/bin/env python
import time
import subprocess
import signal

proc1 = subprocess.Popen(['rosrun', 'my_pcl', 'scan_scene_to_pcd', 'input:=/camera/depth/points'])
time.sleep(3)
proc1.kill()
time.sleep(3)

#proc2 = subprocess.Popen(['rosrun', 'sh_ctrl', 'target_pose_estimation', 'model_draw.pcd', 'cube5cm2.pcd'])
