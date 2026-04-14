#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def set_random_pose():
    rospy.init_node('random_amcl_pose_setter', anonymous=True)

    # 1. 等待地图话题 (只读取一次)
    rospy.loginfo("正在等待地图话题 /map ...")
    try:
        map_data = rospy.wait_for_message('/map', OccupancyGrid, timeout=10)
    except rospy.ROSException:
        rospy.logerr("读取地图超时，请检查 /map 话题是否存在！")
        return

    rospy.loginfo("成功接收到地图，开始计算随机位置...")

    # 2. 解析地图参数
    width = map_data.info.width
    height = map_data.info.height
    res = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y
    
    # 3. 筛选空闲空间 (data == 0)
    # data数组是一维的，索引对应 (y * width + x)
    data = np.array(map_data.data)
    free_indices = np.where(data == 0)[0]

    if len(free_indices) == 0:
        rospy.logerr("地图上没有找到空闲区域(0)！")
        return

    # 4. 随机选一个点
    target_idx = random.choice(free_indices)
    
    # 将一维索引转回网格坐标 (x_grid, y_grid)
    grid_x = target_idx % width
    grid_y = target_idx // width

    # 将网格坐标转换回世界坐标 (meters)
    # 加 0.5*res 是为了移动到像素的中心
    world_x = grid_x * res + origin_x + (0.5 * res)
    world_y = grid_y * res + origin_y + (0.5 * res)

    # 5. 随机生成航向角 (0 到 2*PI)
    random_yaw = random.uniform(0, 2 * np.pi)
    q = quaternion_from_euler(0, 0, random_yaw)

    # 6. 构建 PoseWithCovarianceStamped 消息
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    
    # 稍等一下确保发布者已连接
    rospy.sleep(1.0)

    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"

    # 设置位置
    pose_msg.pose.pose.position.x = world_x
    pose_msg.pose.pose.position.y = world_y
    pose_msg.pose.pose.orientation.x = q[0]
    pose_msg.pose.pose.orientation.y = q[1]
    pose_msg.pose.pose.orientation.z = q[2]
    pose_msg.pose.pose.orientation.w = q[3]

    # 设置协方差 (标准偏差，amcl通常默认使用 0.25)
    # 这里给一个较小的初始不确定性
    cov = [0.0] * 36
    """ cov[0] = 0.25  # x
    cov[7] = 0.25  # y
    cov[35] = 0.06 # yaw (pi/12 * pi/12) """

    cov[0] = 0.25  # x
    cov[7] = 0.25  # y
    cov[35] = 0.25 # yaw (pi/12 * pi/12)

    pose_msg.pose.covariance = cov

    # 7. 发布并退出
    pub.publish(pose_msg)
    rospy.loginfo("随机初始位置已发送: x=%.2f, y=%.2f, yaw=%.2f", world_x, world_y, random_yaw)
    
    # 确保 AMCL 接收到消息
    rospy.sleep(1.0)
    rospy.loginfo("程序运行结束。")

if __name__ == '__main__':
    try:
        set_random_pose()
    except rospy.ROSInterruptException:
        pass