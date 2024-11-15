#! /bin/bash 
# 当需要修改PX4参数时，就必须清除PX4日志，否则修改不会生效。
# Step 1: Start the simulation environment
gnome-terminal -- bash -c "roslaunch px4 outdoor3.launch; exec bash"
sleep 5

# Step 2: Change directory to where attack.py and attack_config.xml are located
cd /home/ros/Attacker/MAVProxy

# Step 3: Start MAVProxy with the current directory set correctly
gnome-terminal -- bash -c "source /home/ros/anaconda3/etc/profile.d/conda.sh; conda activate mavproxy; \
python mavproxy.py --master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560 --cmd='module load attack'; exec bash"
sleep 5

# Step 4：Start QGC
gnome-terminal -- bash -c "~/QGroundControl.AppImage"