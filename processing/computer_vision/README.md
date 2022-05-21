

# Computer Vision docs

# Overview
Python code for a car sim <br/>
Moving_car: Code that simulates a self-driving car, aiming to travel along a line of cones <br/>
coneconnecting: Code that holds math for finding cones that 'connect' to form the track boundry <br/>

# How to use Cone_Detection_Module 

## Implementation of Cone_Detection_Module using YOLOV5 (Current version used for HRD05):

### 1. Make sure you have installed the following:
```
1: Python (At least 3.5)
2: pip
3: CUDA and CUDNN ~If a Nvidia GPU is used to run the model~ (Which is recomended)
4: PyTorch for GPU (https://pytorch.org/get-started/locally/)
5: OpenCV
6: Pandas
7: Various packages may need to be installed 
```
### 2. Open the YOLOv5 folder within the Cone_Detection_Module
Make sure you have the *best.pt* file in the same folder
#### Run the command: 
```
python YOLOv5_Detection.py
```
Happy detection! 🏎️ 

## In case YOLOV4 implementation of the Cone_Detection_Module is wanted follow these steps:  
  
For proper use of the Object_Detection_Module you need to follow these steps:

### 1. Download the configurations and the weights from the HARD Google Drive:
```
https://drive.google.com/drive/folders/16qPjcqzjfw3wzPSjmJlqCNKbgekR5UpY?usp=sharing
```
### 2. Make the following 2 directories:
```
Configurations
```
Where all the contents of the Configurations folder from drive should be in. </br>

And
```
Weights
```
Where the contents of YOLO_V4 Tiny Weights.zip should be in. </br>

In the end you should end up with this folder structure:
```
|New Version
|-Configurations
|-Weights
|-coco.names
|-detecion.py
```
### 3. Make sure you have OpenCV built and optionally with CUDA and CuDNN.
