#-*- coding: utf-8 -*-

import cv2
import time
import argparse
import random
import numpy as np
import onnxruntime as ort

names = ["duck"]

class ONNX_engine():
    def __init__(self, weights, size, cuda) -> None:
        self.img_new_shape = (size, size)
        self.weights = weights
        self.device = cuda
        self.init_engine()
        self.names = names
        self.colors = {name: [random.randint(0, 255) for _ in range(3)] for i, name in enumerate(self.names)}

    def init_engine(self):
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if self.device else ['CPUExecutionProvider']
        self.session = ort.InferenceSession(self.weights, providers=providers)

    def predict(self, im):
        outname = [i.name for i in self.session.get_outputs()]
        inname = [i.name for i in self.session.get_inputs()]
        inp = {inname[0]: im}
        outputs = self.session.run(outname, inp)[0]
        # print(outputs.shape)
        return outputs

    def letterbox(self, im, color=(114, 114, 114), auto=True, scaleup=True, stride=32):
        # 调整大小和垫图像，同时满足跨步多约束
        shape = im.shape[:2]  # current shape [height, width]
        new_shape = self.img_new_shape

        # 如果是单个size的话，就在这里变成一双
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # 尺度比 (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # 只缩小，不扩大(为了更好的val mAP)
            r = min(r, 1.0)

        # 计算填充
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

        if auto:  # 最小矩形区域
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        im = im.transpose((2, 0, 1))
        im = np.expand_dims(im, 0)
        im = np.ascontiguousarray(im)
        im = im.astype(np.float32)
        im /= 255
        return im, r, (dw, dh)
