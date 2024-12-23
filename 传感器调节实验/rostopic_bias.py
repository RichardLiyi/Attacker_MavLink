#!/usr/bin/env python
import rospy
from mavros_msgs.msg import Altitude

class AltitudeBiasNode:
    def __init__(self):
        # 可配置的偏置值
        self.relative_bias = rospy.get_param('~relative_bias', 1.0)
        self.amsl_bias = rospy.get_param('~amsl_bias', 0.0)
        self.local_bias = rospy.get_param('~local_bias', 0.0)
        
        # 创建发布者
        self.pub = rospy.Publisher('/iris_0/mavros/altitude', Altitude, queue_size=10)
        
        # 创建订阅者
        rospy.Subscriber('/iris_0/mavros/altitude', Altitude, self.altitude_callback)
        
        rospy.loginfo("高度偏置节点已启动")
        rospy.loginfo("相对高度偏置: %f", self.relative_bias)
        rospy.loginfo("AMSL高度偏置: %f", self.amsl_bias)
        rospy.loginfo("本地高度偏置: %f", self.local_bias)

    def altitude_callback(self, data):
        try:
            # 保存原始值
            original_relative = data.relative
            original_amsl = data.amsl
            original_local = data.local
            
            # 应用偏置
            data.relative = original_relative + self.relative_bias
            data.amsl = original_amsl + self.amsl_bias
            data.local = original_local + self.local_bias
            
            # 详细日志输出
            rospy.loginfo("高度数据更新:")
            rospy.loginfo("相对高度: %f -> %f", original_relative, data.relative)
            rospy.loginfo("AMSL高度: %f -> %f", original_amsl, data.amsl)
            rospy.loginfo("本地高度: %f -> %f", original_local, data.local)
            
            # 发布更新后的消息
            self.pub.publish(data)
            
        except Exception as e:
            rospy.logerr("处理高度数据时出错: %s", str(e))

if __name__ == '__main__':
    try:
        # 初始化节点
        rospy.init_node('altitude_bias_node')
        
        # 创建节点实例
        node = AltitudeBiasNode()
        
        # 保持节点运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
