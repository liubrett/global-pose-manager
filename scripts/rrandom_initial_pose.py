#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def set_nearby_random_pose():
    rospy.init_node('targeted_chaos_pose_setter', anonymous=True)

    # 1. 监听当前机器人的真实位姿话题
    rospy.loginfo("正在等待当前位置话题 /robot/location1 ...")
    try:
        # 注意：这里假设你发布的 /robot/location 类型是 PoseWithCovarianceStamped
        # 如果你的类型是 geometry_msgs/PoseStamped，请修改这里的类型以及下方的解析代码
        current_pose_msg = rospy.wait_for_message('/robot/location1', PoseWithCovarianceStamped, timeout=10)
    except rospy.ROSException:
        rospy.logerr("读取位置超时，请检查 /robot/location 话题是否存在且正在发布数据！")
        return

    # 2. 解析当前真实坐标
    current_x = current_pose_msg.pose.pose.position.x
    current_y = current_pose_msg.pose.pose.position.y
    rospy.loginfo("成功获取当前真实位置：x=%.2f, y=%.2f", current_x, current_y)

    # 3. 计算 1~3 米的随机偏移量 (使用极坐标转换)
    random_dist = random.uniform(1.0, 3.0)       # 半径：1.0 到 3.0 米
    offset_angle = random.uniform(0, 2 * math.pi) # 角度：0 到 360 度 (2PI)

    # 计算遭受干扰后的新坐标
    target_x = current_x + random_dist * math.cos(offset_angle)
    target_y = current_y + random_dist * math.sin(offset_angle)

    # 4. 随机生成一个坑人的航向角 (0 到 360度)
    random_yaw = random.uniform(0, 2 * math.pi)
    q = quaternion_from_euler(0, 0, random_yaw)

    # 5. 构建发给 AMCL 的毒药数据 (/initialpose)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    
    # 稍等一下确保 Publisher 与 AMCL 建立连接
    rospy.sleep(1.0)

    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"

    # 注入篡改后的位置和姿态
    pose_msg.pose.pose.position.x = target_x
    pose_msg.pose.pose.position.y = target_y
    pose_msg.pose.pose.orientation.x = q[0]
    pose_msg.pose.pose.orientation.y = q[1]
    pose_msg.pose.pose.orientation.z = q[2]
    pose_msg.pose.pose.orientation.w = q[3]

    # 注入庞大的协方差（继承你之前的 2.0 设定，模拟退化的假定）
    cov = [0.0] * 36
    cov[0] = 0.25  # x
    cov[7] = 0.25  # y
    cov[35] = 0.25 # yaw
    pose_msg.pose.covariance = cov

    # 6. 发射毒药数据
    pub.publish(pose_msg)
    rospy.loginfo("========================================")
    rospy.loginfo("【定向干扰发射成功】")
    rospy.loginfo("与原坐标偏移距离: %.2f 米", random_dist)
    rospy.loginfo("发出的毒药坐标: x=%.2f, y=%.2f", target_x, target_y)
    rospy.loginfo("发出的毒药朝向: %.2f 度", math.degrees(random_yaw))
    rospy.loginfo("========================================")
    
    # 确保节点存活足够长时间让 AMCL 收到消息
    rospy.sleep(1.0)
    rospy.loginfo("脚本执行完毕。")

if __name__ == '__main__':
    try:
        set_nearby_random_pose()
    except rospy.ROSInterruptException:
        pass