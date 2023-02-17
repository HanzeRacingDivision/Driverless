# How to use/compile/build the computer vision module (Main module)
## Library dependencies
- cmake >= 3.4
- C/C++14 compiler

## Submodules
This project uses one submodule and we need to initialize and update it
```
git submodule update --init --recursive
```

## Prerequisites

If OpenVINO is installed then no further action is required.

Otherwise install one of the followring, depending on the machine 

 - Ubuntu 20.04/22.04 - `sudo apt install libopencv-dev`
 - MacOS - `brew install opencv`
 - Or install from package / sources (Windows and other OSes)
https://docs.opencv.org/master/d0/d3d/tutorial_general_install.html

## Build DepthAI-Core library
```
cd depthai-core
```
```
git submodule update --init --recursive
```
### Building the library 
```
cmake -S. -Bbuild
cmake --build build
```
More documentation can be found at:
https://github.com/luxonis/depthai-core

Then we head back to the main directory
```
cd ..
```

## Building 
```
mkdir -p build && cd build
cmake ..
cmake --build . --parallel
```
## Runing 
```
./detectionModule
```

# How to use the computer vision module (Demo module)
## !!! This module is just for demonstation purpose !!!

## 1. Installing the dependencies

Refer to the DepthAI docs: https://docs.luxonis.com/en/latest/pages/tutorials/first_steps/#first-steps-with-depthai

## 2. Running the module by itself

```
$ cd Driverless/computer_vision/YOLOv5/scripts
$ python3 cvReworked.py
```

## 3. Understanding the folder configuration

We currently work on the files in the folder "Yolov5."
```
$ cd Driverless/computer_vision/YOLOv5
```

Here we have the following structure:

```
|YOLOv5
|-details
|-pts
|-src
|-shaves
```

### "details" folder
We store the JSON files of the models we test in the details folder. You can find some examples of "detail" files at the moment.

### "pts" folder
In the pts folder, we store the .pt files. These files are the raw file that results from a training session for a model.
To train a model, we use the following jupiter notebook: https://github.com/luxonis/depthai-ml-training/blob/master/colab-notebooks/YoloV5_training.ipynb.

For training a model on a local machine with an Nvidia GPU, refer to this doc: TBA.

After the files are trained, and it results in a .pt file, we use this online transformer: http://tools.luxonis.com/.
As the input shape, put the size of the model (The default size we use is: 416)

The files resulting from the conversion are in a .zip file that will be downloaded from the site. 

From all the files inside the .zip is the .json, which always goes to the "details" folder, and the .blob file, which goes to the "shaves" folder.

### "src" folder
Here we have all the code that we use for the model.

The only file currently in use is "cvReworked.py." All other files are for reference.
To run the code, use the following: 
```
$ python3 cvReworked.py
```

### "shaves" folder
Here are saved all the .blob results from the conversion of the .pt file.


