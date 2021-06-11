import sys
# sys.path.append("f:/lib/site-packages")
# sys.path.append("..")
# sys.path.append(".")

import torch
import numpy as np
import torchvision
from torchvision import datasets,transforms,models
import matplotlib.pyplot as plt
import time
import os
import copy
import torch.nn as nn
from torch import optim
import torch.nn.functional as F
from PIL import Image

# "model_path"        : 'models/0506loss=0.010.tar',

class CNN(object):
    _defaults = {
        "model_path"        : 'E:/zkcup0226/Arduino/v7.0/onRapberry/models/0506loss=0.010.tar',
        "input_size"        : 224,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    #---------------------------------------------------#
    #   初始化CNN
    #---------------------------------------------------#
    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults)
        self.class_names = ['beer', 'bluecube', 'magiccube','redbull','redcube']
        self.num_classes = len(self.class_names)
        self.generate()


    #---------------------------------------------------#
    #   生成模型
    #---------------------------------------------------#
    def generate(self):
        #---------------------------------------------------#
        #   建立CNN模型
        #---------------------------------------------------#
        self.model = models.resnet18(pretrained=True)
        num_ftrs = self.model.fc.in_features
        self.model.fc=nn.Linear(num_ftrs,self.num_classes)
        #---------------------------------------------------#
        #   载入CNN模型的权重
        #---------------------------------------------------#
        print("*"*60)
        print('Loading CNN weights into state dict...')
        self.model.load_state_dict(torch.load(self.model_path, map_location=torch.device('cpu')))
        self.model.eval()
        print('Finished!')
        print("*"*60)

        self.to_tensor = transforms.Compose([
                transforms.Resize(256),
                transforms.CenterCrop(self.input_size),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                            std=[0.229, 0.224, 0.225]),
            ])

    def detect_image(self, image):
        image.resize((256,256), Image.BILINEAR)
        image = self.to_tensor(image)
        image = torch.unsqueeze(image, 0)
        torch.set_grad_enabled(False)
        outputs=self.model(image)
        max_, preds_index = torch.max(outputs, 1)
        # print(preds_index)
        pred_softmax = F.softmax(outputs,dim=1)
        max_softmax,_ = torch.max(pred_softmax,1)
        # print(max_softmax)

        # return preds_index[0].numpy(), max_softmax[0].numpy()   # 将数据输出为numpy.array格式
        return preds_index[0].tolist(), max_softmax[0].tolist()                    # 将数据输出为普通格式

        
        

if __name__ == "__main__":
    
    cnn = CNN()

    while True:
        img = input('Input image filename:')
        try:
            image = Image.open(img)
        except:
            print('Open Error! Try again!')
            continue
        else:
            preds_result, confidence = cnn.detect_image(image)
            print(preds_result)
            print(confidence)

