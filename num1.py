#!/usr/bin/env python
# encoding=utf-8

import pyrealsense2 as rs
import open3d as o3d
import numpy as np
from os import path
import os
from PIL import Image
import sys
import copy
import cv2
from matplotlib import pyplot as plt
import rospy
from PIL import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
align_to = rs.stream.color
align = rs.align(align_to)

#发布2个消息：
#point 消息存储目标点的坐标；
#vector存储目标的法向量，向量方向从目标表面向外指向相机

#在代码同级创建文件夹“out”
save_path = os.path.join(os.getcwd(), "out")
if not os.path.exists(save_path):
    os.mkdir(save_path)
#相机坐标系：设想双目相机为人的双眼，X轴向右红色，Y轴向上绿色，Z轴向后蓝色

class realsense_im(object):
    def __init__(self,image_size=(640,480)):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, image_size[0], image_size[1], rs.format.z16, 30)
        config.enable_stream(rs.stream.color, image_size[0], image_size[1], rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)

    def __get_depth_scale(self):
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        return depth_scale

    def get_image(self):

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_image = np.asarray(depth_frame.get_data(), dtype=np.float32)
        color_image = np.asarray(color_frame.get_data(), dtype=np.uint8)
        color_image_pad = np.pad(color_image, ((20, 0), (0, 0), (0, 0)), "edge")
        depth_map_end = depth_image * self.__get_depth_scale() * 1000
        return depth_map_end,color_image
    def process_end(self):
        self.pipeline.stop()

rs_t=realsense_im()

i=0

def detecte(image):
    #提取所有轮廓
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    _,gray=cv2.threshold(gray,0,255,cv2.THRESH_OTSU+cv2.THRESH_BINARY_INV)
    contours,hierachy=cv2.findContours(gray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #img,contours,hierachy=cv2.findContours(gray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return image,contours,hierachy

def compute_1(contours,i,j):
    #最外面的轮廓和子轮廓的比例
    area1 = cv2.contourArea(contours[i])
    area2 = cv2.contourArea(contours[j])
    if area2==0:
        return False
    ratio = area1 * 1.0 / area2
    if abs(ratio - 49.0 / 25):
        return True
    return False

def compute_2(contours,i,j):
    #'''子轮廓和子子轮廓的比例'''
    area1 = cv2.contourArea(contours[i])
    area2 = cv2.contourArea(contours[j])
    if area2==0:
        return False
    ratio = area1 * 1.0 / area2
    if abs(ratio - 25.0 / 9):
        return True
    return False

def compute_center(contours,i):
    #'''计算轮廓中心点'''
    M=cv2.moments(contours[i])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return cx,cy

def detect_contours(vec):
    #判断轮廓和子轮廓及子子轮廓的中心的间距是否足够小
    distance_1=np.sqrt((vec[0]-vec[2])**2+(vec[1]-vec[3])**2)
    distance_2=np.sqrt((vec[0]-vec[4])**2+(vec[1]-vec[5])**2)
    distance_3=np.sqrt((vec[2]-vec[4])**2+(vec[3]-vec[5])**2)
    if sum((distance_1,distance_2,distance_3))/3<3:
        return True
    return False

def juge_angle(rec):
    #判断寻找是否有三个点可以围成等腰直角三角形
    if len(rec)<3:
        return -1,-1,-1
    for i in range(len(rec)):
        for j in range(i+1,len(rec)):
            for k in range(j+1,len(rec)):
                distance_1 = np.sqrt((rec[i][0] - rec[j][0]) ** 2 + (rec[i][1] - rec[j][1]) ** 2)
                distance_2 = np.sqrt((rec[i][0] - rec[k][0]) ** 2 + (rec[i][1] - rec[k][1]) ** 2)
                distance_3 = np.sqrt((rec[j][0] - rec[k][0]) ** 2 + (rec[j][1] - rec[k][1]) ** 2)
                if abs(distance_1-distance_2)<5:
                    if abs(np.sqrt(np.square(distance_1)+np.square(distance_2))-distance_3)<5:
                        return i,j,k
                elif abs(distance_1-distance_3)<5:
                    if abs(np.sqrt(np.square(distance_1)+np.square(distance_3))-distance_2)<5:
                        return i,j,k
                elif abs(distance_2-distance_3)<5:
                    if abs(np.sqrt(np.square(distance_2)+np.square(distance_3))-distance_1)<5:
                        return i,j,k
    return -1,-1,-1


def talker(avg):
    pub = rospy.Publisher('chatter', Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    avg = avg.astype(np.float64)
    temp=Point()
    #定义Point类的对象
    temp.x=avg[0]
    temp.y=avg[1]
    temp.z=avg[2]
    if not rospy.is_shutdown():
        rospy.loginfo(temp)
        pub.publish(temp)
        return

def talker2(nomavg):
    pub2 = rospy.Publisher('chatter2', Vector3, queue_size=10)
    #rospy.init_node('talker2', anonymous=True)
    nomavg = nomavg.astype(np.float64)
    temp2=Vector3()
    temp2.x=nomavg[0]
    temp2.y=nomavg[1]
    temp2.z=nomavg[2]
    if not rospy.is_shutdown():
        rospy.loginfo(temp2)
        pub2.publish(temp2)
        return


def find(image,contours,hierachy,root=0):
    #找到符合要求的轮廓
    rec=[]
    for i in range(len(hierachy)):
        child = hierachy[i][2]
        child_child=hierachy[child][2]
        if child!=-1 and hierachy[child][2]!=-1:
            if compute_1(contours, i, child) and compute_2(contours,child,child_child):
                cx1,cy1=compute_center(contours,i)
                cx2,cy2=compute_center(contours,child)
                cx3,cy3=compute_center(contours,child_child)
                if detect_contours([cx1,cy1,cx2,cy2,cx3,cy3]):
                    rec.append([cx1,cy1,cx2,cy2,cx3,cy3,i,child,child_child])
    #计算得到所有在比例上符合要求的轮廓中心点
    i,j,k=juge_angle(rec)
    if i==-1 or j== -1 or k==-1:
        print("哎嘛妹找到二维码啊")
        return
    else:
        ts = np.concatenate((contours[rec[i][6]], contours[rec[j][6]], contours[rec[k][6]]))
        rect = cv2.minAreaRect(ts)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        result=copy.deepcopy(image)
        #[box]记录二维码的四个角点
    
        #cv2.drawContours(result, [box], 0, (0, 0, 255), 2)
        #cv2.drawContours(result,contours,rec[i][6],(255,0,0),2)
        #cv2.drawContours(result,contours,rec[j][6],(255,0,0),2)
        #cv2.drawContours(result,contours,rec[k][6],(255,0,0),2)
        #cv2.imshow('result1',result)
        #显示结果，调试用

        pts = np.array([[box][0][0], [box][0][1], [box][0][2], [box][0][3]])
        pts = np.array([pts])
        #mask:与原始图像一样大的黑色板子。640*480 单通道
        mask = np.zeros(image.shape[:2], np.uint8)
        #print("mask.shape:{}".format(mask.shape))
        #白色多边形
        cv2.polylines(mask, pts, 1, 255)
        cv2.fillPoly(mask, pts, 255)
        # 按位与，裁剪，黑色背景
        rgb_cut = cv2.bitwise_and(result, result, mask=mask)
        #cv2.imshow('rgb_cut',rgb_cut)
        depth = cv2.imread("out/depth.png",cv2.IMREAD_ANYDEPTH)##
        #深度图需要按照单通道读入!
        #depth需要读取为单通道图像
        #print("depth.shape:{}".format(depth.shape))
        depth_cut = cv2.bitwise_and(depth, depth, mask=mask)
        #with open("dep_cut.txt","w") as f:
        #    for i in range(480):
        #        for j in range(640):
        #            f.write(str(depth_cut[i][j]))
        qrcode = cv2.QRCodeDetector()
        result, points, code = qrcode.detectAndDecode(rgb_cut)
        print(result)
        cv2.imwrite("out/rgb_cut.png", rgb_cut)
        cv2.imwrite("out/depth_cut.png", depth_cut)
        recreate()
    return
        #cv2.waitKey(0)

def recreate():
    color_raw = o3d.io.read_image("out/rgb_cut.png")
    depth_raw = o3d.io.read_image("out/depth_cut.png")
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
    #rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, convert_rgb_to_intensity=False)

    #调试用窗口，很漂亮
    #plt.subplot(1, 2, 1)
    #plt.title('QR Code Gray Image')
    #plt.imshow(rgbd_image.color)
    #plt.subplot(1, 2, 2)
    #plt.title('QR Code Depth Image')
    #plt.imshow(rgbd_image.depth)
    #plt.show()

    inter = o3d.camera.PinholeCameraIntrinsic()
    inter.set_intrinsics(640, 480, 614.3273315429688, 614.3955078125, 321.7488098144531, 245.80747985839844)
    #相机内参：宽，高，焦距fx，焦距fy，光学中心cx，光学中心cy
    pcd = o3d.geometry.PointCloud().create_from_rgbd_image(rgbd_image, inter)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)#size=2e-06
    #np.savetxt('out/site.txt',np.asarray(pcd.points)) 
    if len(np.asarray(pcd.points)) == 0:
    	return 0
    else:
        avg = sum(np.asarray(pcd.points))/(len(np.asarray(pcd.points))*1.0) 
        #调试用，会占用线程
        #o3d.io.write_point_cloud('out/code.ply',pcd)
        #降采样效果不好
        #down=pcd.voxel_down_sample(voxel_size=0.002)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
        #KD Tree搜索半径5cm，最大处理30个相邻点
        #o3d.visualization.draw_geometries([mesh,pcd],point_show_normal=True)
        vec= np.asarray(pcd.normals)
        m=0
        for i in range(len(vec)):
            if vec[i][2] <= 0:
                #np.delete(vec,[i])
                vec[i]=0
                m=m+1
                #把反向的向量归0，m为归0向量的个数
    
        #np.savetxt('out/nom.txt',vec)
        nomavg = sum(vec)/(len(vec)*1.0-m)
        #求平均时需要去掉0向量的个数
        # try:
        #     talker(avg)
        #     talker2(nomavg)
        # except rospy.ROSInterruptException:
        #     pass
        return 1


if __name__ == '__main__':
    while True:
        key = cv2.waitKey(150)
        depth_map,rgb_map=rs_t.get_image()
        #按s按键拍照
        #if key & 0xFF == ord('s'):
            #print("photo saved")
        cv2.imwrite('out/rgb.png', rgb_map)
        cv2.imwrite('out/depth.png', np.asarray(depth_map,np.uint16))
        i+=1
        image = cv2.imread("out/rgb.png")
        image,contours,hierachy=detecte(image)
        find(image,contours,np.squeeze(hierachy))
            #end of if
        cv2.namedWindow('result', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('result', rgb_map)
        #cv2.namedWindow('depth', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('depth', np.asarray(depth_map,np.uint16))
        #按q按键退出
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break




