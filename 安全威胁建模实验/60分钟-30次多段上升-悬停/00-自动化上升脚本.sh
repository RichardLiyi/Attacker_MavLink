#! /bin/bash 
# 当需要修改PX4参数时，就必须清除PX4日志，否则修改不会生效。

# 步骤1：启动仿真环境
gnome-terminal -- bash -c "roslaunch px4 outdoor3.launch; exec bash"
sleep 5

# 步骤2：启动MAVProxy中继
# python mavproxy.py --master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560 --cmd='module load attack; script auto_attack.scr'; \
gnome-terminal --working-directory=/home/ros/Attacker_MavLink/Attacker/MAVProxy -- bash -c "\
source /home/ros/anaconda3/etc/profile.d/conda.sh; \
conda activate mavproxy; \
python mavproxy.py --master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560 --cmd='module load attack'; \
exec bash"
sleep 5

#步骤4：启动通信中继节点:
gnome-terminal -- bash -c "/usr/bin/python /home/ros/Attacker_MavLink/00-攻击文件/60分钟-30次多段上升-悬停/multirotor_communication.py iris 0"
sleep 5

# 步骤5：无人机飞行控制脚本
gnome-terminal -- bash -c "/usr/bin/python /home/ros/Attacker_MavLink/00-攻击文件/60分钟-30次多段上升-悬停/60min-UpAndHover.py iris 1 vel"
sleep 5