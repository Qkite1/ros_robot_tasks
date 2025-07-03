import os
import numpy as np
import torch
import cv2

import sys
sys.path.insert(0, "/home/q/robot/litchiinst/FastInst-main")

from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
from detectron2.data import MetadataCatalog
from detectron2.projects.deeplab import add_deeplab_config
from fastinst.config import add_fastinst_config

class FastInstEngine:
    def __init__(self, config_file, weights_file, opts=None, confidence_threshold=0.35):
        self.cfg = self._setup_cfg(config_file, weights_file, opts)
        self.predictor = DefaultPredictor(self.cfg)
        self.metadata = MetadataCatalog.get(self.cfg.DATASETS.TEST[0])
        self.conf_thresh = confidence_threshold

    def _setup_cfg(self, config_file, weights_file, opts):
        cfg = get_cfg()
        add_deeplab_config(cfg)
        add_fastinst_config(cfg)
        cfg.merge_from_file(config_file)
        cfg.MODEL.WEIGHTS = weights_file
        if opts is not None:
            cfg.merge_from_list(opts)
        cfg.freeze()
        return cfg

    def predict(self, img):  # 预测接口，传图像
        """
        输入图像，返回置信度大于阈值的所有实例的掩码、类别、置信度等。
        :param img: 输入图像（BGR 或 文件路径）
        :return: list[dict]，每个dict包含 'mask', 'class', 'score'
        """
        if isinstance(img, str):
            img = cv2.imread(img)

        outputs = self.predictor(img)

        if not outputs or "instances" not in outputs:
            return []

        instances = outputs["instances"].to("cpu")
        scores = instances.scores.numpy()
        masks = instances.pred_masks.numpy()
        classes = instances.pred_classes.numpy()

        results = []
        for i in range(len(scores)):
            if scores[i] >= self.conf_thresh:
                result = {
                    "mask": masks[i],
                    "class": self.metadata.thing_classes[classes[i]],
                    "score": float(scores[i]),
                    "class_id": int(classes[i])
                }
                results.append(result)

        return results
