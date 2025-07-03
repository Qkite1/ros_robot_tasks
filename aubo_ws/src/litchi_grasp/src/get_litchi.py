#!/home/q/anaconda3/envs/litchi/bin/python
# -*-coding: utf-8 -*-

from litchi_grasp.srv import *
import rospy
import time
import os
import numpy as np

import pyrealsense2 as rs 

import cv2 
import threading 
from litchi_engine import FastInstEngine
import torch
from detectron2.data import DatasetCatalog, MetadataCatalog
from detectron2.data.datasets import register_coco_instances



position_list = []
count = 0
fastinst_engine = None

CLASS_NAMES = ['background', 'litchi cluster', 'main fruit bearing']

DATASET_ROOT = './datasets/litchi_data'
ANN__ROOT = os.path.join(DATASET_ROOT, 'annotations')
TRAIN_PATH = os.path.join(DATASET_ROOT)
VAL_PATH = os.path.join(DATASET_ROOT)
TRAIN_JSON = os.path.join(ANN__ROOT, 'instances_litchitrain.json')
VAL_JSON = os.path.join(ANN__ROOT, 'instances_litchival.json')

PREDEFINED_SPLITS_DATASET = {
    "litchi_train": (TRAIN_PATH, TRAIN_JSON),
    "litchi_val": (VAL_PATH, VAL_JSON),
}

def plain_register_dataset():
    DatasetCatalog.register("litchi_train", lambda: load_coco_json(TRAIN_JSON, TRAIN_PATH))
    MetadataCatalog.get("litchi_train").set(thing_classes=CLASS_NAMES,
                                             evaluator_type='coco',
                                             json_file=TRAIN_JSON,
                                             image_root=TRAIN_PATH)
    
    DatasetCatalog.register("litchi_val", lambda: load_coco_json(VAL_JSON, VAL_PATH))
    MetadataCatalog.get("litchi_val").set(thing_classes=CLASS_NAMES,
                                             evaluator_type='coco',
                                             json_file=VAL_JSON,
                                             image_root=VAL_PATH)

# ====相机帧获取===
def get_aligned_images():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    aligned_color_frame = aligned_frames.get_color_frame()
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
    img_color = np.asanyarray(aligned_color_frame.get_data()) 
    img_depth = np.asanyarray(aligned_depth_frame.get_data())
    return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame

#======掩码及中心坐标获取===

def get_mask_center(img_color, engine):
    results = engine.predict(img_color)
    #print("Detection results:", results)

    if not results:
        #print("No instances detected.")
        return 320, 240, None

    # 寻找 class_id 为 2（main fruit bearing）的目标
    target_class_id = 2
    filtered = [r for r in results if r["class_id"] == target_class_id and r["score"] >= 0.3]

    if not filtered:
        return 320, 240, None

    # 选择置信度最高的那个
    best = max(filtered, key=lambda x: x["score"])
    mask = best["mask"]

    y_idx, x_idx = np.where(mask)
    if len(x_idx) == 0 or len(y_idx) == 0:
        return 320, 240, None

    center_x = int(np.mean(x_idx))
    center_y = int(np.mean(y_idx))

    return center_x, center_y, mask


#=======相机坐标计算======
def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = depth_pixel[0]
    y = depth_pixel[1]
    dis = aligned_depth_frame.get_distance(x, y)
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    return dis, camera_coordinate

#======坐标变化判定=====
STABLE_THRESH = 5
def is_coord_similar(coord1, coord2, thresh=STABLE_THRESH):
    return all(abs(a-b) < thresh for a, b in zip(coord1, coord2))

#======图像处理线程========
def handle_imagestream_thread():
    count = 0
    while True:
        color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images()  
        x, y, mask = get_mask_center(img_color, fastinst_engine)
        depth_pixel = [x, y]
        dis, camera_coordinate = get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin)
        vis = img_color.copy()
        if mask is not None:
    # 将掩码转为3通道 uint8 格式
            mask_uint8 = (mask.astype(np.uint8)) * 255
            mask_3c = cv2.merge([mask_uint8, mask_uint8, mask_uint8])

    # 创建红色图层（带透明度）
            red_layer = np.zeros_like(img_color, dtype=np.uint8)
            red_layer[:, :, 2] = 255  # 红色通道设置为255

    # 掩码区域叠加红色（alpha混合）
            alpha = 0.5  # 透明度系数
            vis = img_color.copy()
            vis = np.where(mask_3c > 0, cv2.addWeighted(vis, 1 - alpha, red_layer, alpha, 0), vis)
        cv2.circle(img_color, (x, y), 8, [255, 0, 255], thickness=-1)
        cv2.putText(img_color, "Dis:" + str(dis) + " m", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, [0, 0, 255])
        cv2.putText(img_color, "X:" + str(camera_coordinate[0]) + " m", (80, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    [255, 0, 0])
        cv2.putText(img_color, "Y:" + str(camera_coordinate[1]) + " m", (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    [255, 0, 0])
        cv2.putText(img_color, "Z:" + str(camera_coordinate[2]) + " m", (80, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    [255, 0, 0])
        
        cv2.imshow('RealSence', img_color)
        cv2.imshow('Mask Overlay', vis)
        
        depth_colormap = cv2.convertScaleAbs(img_depth, alpha=0.03)
        depth_colormap = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)
        cv2.imshow('Depth Map (Colored)', depth_colormap)
        
        key = cv2.waitKey(1)

        if camera_coordinate[2] != 0:
            if len(position_list) < 10:
                position_list.append(camera_coordinate)
            else:
                position_list.pop(0)
                position_list.append(camera_coordinate)


#======ros回调===========
def return_position_list(req):
    global count
    print(req.name)
    count += 1

    valid_positions = [pos for pos in position_list if not (
        abs(pos[0])<1e-3 and abs(pos[1]) < 1e-3 and abs(pos[2]) < 1e-3
    )]

    if valid_positions:
        last_valid = valid_positions[-1]
        position_list.clear()
        count = 0
        return LocatedLitchiResponse(last_valid)

    time.sleep(0.5)
    return LocatedLitchiResponse([0, 0, 0])
    # if len(position_list) == 10:
    #     # 排除中心点 (320, 240) 对应的相机坐标（可能是默认0点）
    #     for pos in position_list:
    #         if abs(pos[0]) < 1e-3 and abs(pos[1]) < 1e-3 and abs(pos[2]) < 1e-3:
    #             print("检测到无效中心点坐标，拒绝发送")
    #             position_list.clear()
    #             count = 0
    #             return LocatedLitchiResponse([0, 0, 0])

    #     # 判断坐标是否稳定
    #     max_diff = 0
    #     for i in range(1, len(position_list)):
    #         diff = np.linalg.norm(np.array(position_list[i]) - np.array(position_list[i-1]))
    #         max_diff = max(max_diff, diff)
        
    #     if max_diff < 0.15:  # <5cm 波动算稳定
    #         avg = np.mean(position_list, axis=0)
            
    #         # #避免深度值不稳定干扰
    #         # valid_z_values = [pos[2] for pos in position_list if pos[2] > 1e-3 ]
    #         # if valid_z_values:
    #         # 	z_min = min(valid_z_values)
    #         # 	avg[2] = z_min
            	
    #         position_list.clear()
    #         count = 0
    #         return LocatedLitchiResponse(avg.tolist())
    #     else:
    #         print("坐标不稳定，拒绝发送")
    #         position_list.clear()
    #         count = 0
    #         return LocatedLitchiResponse([0, 0, 0])

    # else:
    #     time.sleep(2)
    #     if count >= 5:
    #         count = 0
    #         print("数据不足，超时返回")
    #         return LocatedLitchiResponse([0, 0, 0])

#=========ros节点================
def located_litchi_server():
    rospy.init_node("located_litchi_server", anonymous=True)
    threading.Thread(target=handle_imagestream_thread, args=()).start()
    s = rospy.Service('located_litchi', LocatedLitchi, return_position_list)
    rospy.spin()

if __name__ =="__main__":

    plain_register_dataset()
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
    pipe_profile = pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)
    config_file = "/home/q/robot/aubo_ws/src/litchi_grasp/src/litchiinst/withsim.yaml"
    weights_file = "/home/q/robot/aubo_ws/src/litchi_grasp/src/litchiinst/withsim.pth"
    fastinst_engine = FastInstEngine(config_file, weights_file)
    located_litchi_server()

