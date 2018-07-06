#!/usr/bin/python3

from imageai.Prediction.Custom import CustomImagePrediction
import numpy as np
import os

execution_path = os.path.join(os.path.split(os.getcwd())[0],'data')
IMAGE_HEIGHT = 50
IMAGE_WIDTH  = 50
rtVal={'b':[0],'e':[1],'f':[2],'x':[3]}

# load model
prediction = CustomImagePrediction()
prediction.setModelTypeAsResNet()
prediction.setModelPath(os.path.join(execution_path, "model_ex-182_acc-1.000000.h5"))
prediction.setJsonPath(os.path.join(execution_path, "model_class.json"))
prediction.loadModel(num_objects=4)

def main(imgList):
    img = np.array(imgList,dtype=np.uint8)
    img = np.reshape(img,(IMAGE_HEIGHT,IMAGE_WIDTH,3))
    predictions, probabilities = prediction.predictImage(img,input_type='array', result_count=1)
    
    return tuple(rtVal[predictions[0]])
