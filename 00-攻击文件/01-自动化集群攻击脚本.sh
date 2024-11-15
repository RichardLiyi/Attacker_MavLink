#! /bin/bash 
# 当需要修改PX4参数时，就必须清除PX4日志，否则修改不会生效。

# 步骤1：启动仿真环境
gnome-terminal -- bash -c "roslaunch px4 iris5_backbone_exam.launch"
sleep 5

# 步骤2：启动MAVProxy中继
# python mavproxy.py --master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560 --cmd='module load attack; script auto_attack.scr'; \
# gnome-terminal --working-directory=/home/ros/Attacker/MAVProxy -- bash -c "\
# source /home/ros/anaconda3/etc/profile.d/conda.sh; \
# conda activate mavproxy; \
# python mavproxy.py --master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560 --cmd='module load attack'; \
# exec bash"

n=5  # 终端数量
base_in_port=4660  # 起始端口号
base_out_port=4560  # 起始端口号

for i in $(seq 0 $(($n - 1))); do
    port_in=$(($base_in_port + $i))
    port_out=$(($base_out_port+$i))  # 假设 out 端口是 in 端口减去 100

    gnome-terminal --working-directory=/home/ros/Attacker/MAVProxy -- bash -c "\
    source /home/ros/anaconda3/etc/profile.d/conda.sh; \
    conda activate mavproxy; \
    python mavproxy.py --master=tcpin:127.0.0.1:$port_in --out=tcp:127.0.0.1:$port_out --cmd='module load attack'; \
    exec bash" &
done

sleep 12

#步骤4：启动通信中继节点:
gnome-terminal -- bash -c "mpiexec -n 5 /usr/bin/python /home/ros/catkin_ws/src/control/script/sim_addleader_new.py"

sleep 5


# 步骤5：无人机飞行控制脚本
gnome-terminal -- bash -c "/usr/bin/python /home/ros/catkin_ws/src/control/script/gcs_control.py"
sleep 5