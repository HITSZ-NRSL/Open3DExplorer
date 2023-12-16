#!/usr/bin/env python
#coding=utf-8


from cmath import pi
import rospy
from nav_msgs.msg import OccupancyGrid  # map 数据
import numpy as np

Limit_range = 20

# right_limit = -5
# left_limit = 22.5
# bottom_limit = -5
# up_limit = 25

# from cmath import pi
# yj = -2.8
# yj1 = 2.8
# if abs(yj1 - yj) > pi:
#   yj1 = (pi + (pi+yj1)) if yj > 0 else (- pi - (pi-yj1))
# print('yj: ', yj, 'yj1: ', yj1 )

class TestMap(object):
    def __init__(self):
        # Give the node a name
        rospy.init_node('coverage_count', anonymous=False)

        rospy.Subscriber("/projected_map2d", OccupancyGrid, self.get_map_data, queue_size=10)

    def get_map_data(self, msg):
        height = msg.info.height  # pgm 图片属性的像素值（栅格地图初始化大小——高度上的珊格个数）
        width = msg.info.width  # pgm 图片属性的像素值（栅格地图初始化大小中的宽——宽度上的珊格个数）
        origin_x = msg.info.origin.position.x  # ROS 建图的 origin
        origin_y = msg.info.origin.position.y  # ROS 建图的 origin
        resolution = msg.info.resolution  # ROS 建图的分辨率 resolution（栅格地图分辨率对应栅格地图中一小格的长和宽）
        data_size = len(msg.data)
        time_stamp = msg.header.seq
        # print time_stamp
        # print(height, width, origin_x, origin_y, resolution, data_size)
# (499, 499, -100.00000305473804, -100.00000305473804, 0.4000000059604645, 249001)

        mapdata = msg.data
        # print type(mapdata)

        mapdata = np.array(mapdata)
        # print mapdata.shape

        mapdata = mapdata.reshape((height, width))

        occupied = 0
        for i in range(height):
          for j in range(width):
            if((abs((i - height*0.5)*resolution) < Limit_range) and (abs((j - height*0.5)*resolution) < Limit_range) and ((mapdata[i,j]==100) or (mapdata[i,j]==-100))):
              occupied = occupied + 1
              print("index:", i, j)

        # for i in range(height):
        #   for j in range(width):
        #     if((((i - height*0.5)*resolution) > right_limit) and (((i - height*0.5)*resolution) < left_limit) and (abs((j - height*0.5)*resolution) > bottom_limit) and (abs((j - height*0.5)*resolution) < up_limit) and ((mapdata[i,j]==100) or (mapdata[i,j]==-100))):
        #       occupied = occupied + 1
        #       # print("index:", i, j)


        total_occupied = ((2*Limit_range)/0.4)*((2*Limit_range)/0.4)

        # total_occupied = ((left_limit - right_limit)/resolution)*((up_limit - bottom_limit)/resolution)
        # print total_occupied
        print occupied/total_occupied
        
        # 保存数据为 txt 文件
        with open('data.txt', 'a') as f:
            f.write(str(occupied/total_occupied) + " " + str(time_stamp) + "\n")
       	# print('保存成功')


if __name__ == "__main__":
    try:
        # open("data.txt", 'w').close()
        TestMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Test Map terminated.')
