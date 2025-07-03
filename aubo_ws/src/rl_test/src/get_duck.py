#!/usr/bin/env python
# -*-coding: utf-8 -*-

from rl_test.srv import *
import rospy
import time

import numpy as np

import pyrealsense2 as rs 

import cv2 
import threading 
from duck_engine import ONNX_engine


model_path ="/home/q/MADD/aubo_ws/src/rl_test/src/yolov8n-duck-best.onnx"
global_engine = None
position_list = []
count = 0

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

def postprocess(im, output, confidence_thres=0.25, iou_thres=0.7):
    input_width = 640
    input_height = 640
    img_width = im.shape[2]
    img_height = im.shape[3]

    # Transpose and squeeze the output to match the expected shape
    outputs = np.transpose(np.squeeze(output[0]))

    # Get the number of rows in the outputs array
    rows = outputs.shape[0]

    # Lists to store the bounding boxes, scores, and class IDs of the detections
    boxes = []
    scores = []
    class_ids = []

    # Calculate the scaling factors for the bounding box coordinates
    x_factor = img_width / input_width
    y_factor = img_height / input_height

    # Iterate over each row in the outputs array
    for i in range(rows):
        # Extract the class scores from the current row
        classes_scores = outputs[i][4:]

        # Find the maximum score among the class scores
        max_score = np.amax(classes_scores)

        # If the maximum score is above the confidence threshold
        if max_score >= confidence_thres:
            # Get the class ID with the highest score
            class_id = np.argmax(classes_scores)

            # Extract the bounding box coordinates from the current row
            x, y, w, h = outputs[i][0], outputs[i][1], outputs[i][2], outputs[i][3]

            # Calculate the scaled coordinates of the bounding box
            left = int((x - w / 2) * x_factor)
            top = int((y - h / 2) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)

            # Add the class ID, score, and box coordinates to the respective lists
            class_ids.append(class_id)
            scores.append(max_score)
            boxes.append([left, top, width, height])

    # Apply non-maximum suppression to filter out overlapping bounding boxes
    indices = cv2.dnn.NMSBoxes(boxes, scores, confidence_thres, iou_thres)


    alls = []
    # Iterate over the selected indices after non-maximum suppression
    for i in indices:
        # Get the box, score, and class ID corresponding to the index
        box = boxes[i]
        score = scores[i]
        class_id = class_ids[i]
        float_list = [float(num) for num in box]
        x, y, w, h = float_list
        x0 = x 
        y0 = y 
        x1 = x + w 
        y1 = y + h
        alls.append([0, x0, y0, x1, y1, class_id, score])
        # Draw the detection on the input image
        # self.draw_detections(input_image, box, score, class_id)
        kk = 0
    # Return the modified input image
    return alls

def get_duck(img_color):
    global global_engine
    img = cv2.cvtColor(img_color, cv2.COLOR_BGR2RGB)
    image = img.copy()
    im, ratio, dwdh = global_engine.letterbox(image, auto=False)
    outputs_ori = global_engine.predict(im)
    outputs = postprocess(im, outputs_ori)
    ori_images = [img_color.copy()]


    if len(outputs)>0:
        batch_id, x0, y0, x1, y1, cls_id, score = outputs[0]
        image = ori_images[int(batch_id)]
        box = np.array([x0, y0, x1, y1])
        box -= np.array(dwdh * 2)
        box /= ratio
        box = box.round().astype(np.int32).tolist()
        cls_id = int(cls_id)
        score = round(float(score), 3)
        name = global_engine.names[cls_id]
        color = global_engine.colors[name]
        name += ' ' + str(score)
        cv2.rectangle(image, box[:2], box[2:], color, 2)
        cv2.putText(image, name, (box[0], box[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, [225, 255, 255], thickness=1)
        depth_pixel = [int((box[2]-box[0])/2+box[0]), int((box[3]-box[1])/2+box[1])] 
        output_depth_pixel = depth_pixel
    else:
        output_depth_pixel = [320, 240]

    cv2.imshow("capture", ori_images[0])
    return output_depth_pixel[0], output_depth_pixel[1]

def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = depth_pixel[0]
    y = depth_pixel[1]
    dis = aligned_depth_frame.get_distance(x, y)
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    return dis, camera_coordinate

def handle_imagestream_thread():
    count = 0
    while True:
        color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images()  
        x, y = get_duck(img_color)
        depth_pixel = [x, y]
        dis, camera_coordinate = get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin)

        cv2.circle(img_color, (x, y), 8, [255, 0, 255], thickness=-1)
        cv2.putText(img_color, "Dis:" + str(dis) + " m", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, [0, 0, 255])
        cv2.putText(img_color, "X:" + str(camera_coordinate[0]) + " m", (80, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    [255, 0, 0])
        cv2.putText(img_color, "Y:" + str(camera_coordinate[1]) + " m", (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    [255, 0, 0])
        cv2.putText(img_color, "Z:" + str(camera_coordinate[2]) + " m", (80, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    [255, 0, 0])
        
        cv2.imshow('RealSence', img_color)
        key = cv2.waitKey(1)

        if camera_coordinate[2] != 0:
            if len(position_list) < 10:
                position_list.append(camera_coordinate)
            else:
                position_list.pop(0)
                position_list.append(camera_coordinate)

def return_position_list(req):
    global count
    print(req.name)
    sum_x = 0
    sum_y = 0
    sum_z = 0
    count += 1
    if len(position_list) == 10:
        for p in position_list:
            sum_x += p[0]
            sum_y += p[1]
            sum_z += p[2]
            count = 0
        return LocatedDuckResponse([sum_x/10, sum_y/10,sum_z /10])
    else:
        time.sleep(2)
        if count == 5:
            count  = 0
            return LocatedDuckResponse([0,0,0])

def located_duck_server():
    rospy.init_node("located_duck_server", anonymous=True)
    threading.Thread(target=handle_imagestream_thread, args=()).start()
    s = rospy.Service('located_duck', LocatedDuck, return_position_list)
    rospy.spin()

if __name__ =="__main__":
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
    pipe_profile = pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)
    global_engine = ONNX_engine(model_path, 640, False)
    located_duck_server()

