#include <chrono>
#include <fstream>
#include <iostream>
#include <string.h>
#include <nlohmann/json.hpp>
#include <depthai/depthai.hpp>

// Includes common necessary includes for development using depthai library

#define BLOB_PATH "../shaves/bestFSN_openvino_2021.4_6shave.blob"

static std::atomic<bool> syncNN{true};

int main(int argc, char **argv)
{
     using namespace std::chrono;
     using json = nlohmann::json;

     std::ifstream f("../details/bestFSN.json", std::ifstream::in);
     json j;
     f >> j;

     // Getting the labels for the model
     std::vector<std::string> labels = j["mappings"]["labels"];
     std::vector<std::string> labelMap = labels;

     // Getting the input size 
     std::string inputSizeStr = j["nn_config"]["input_size"];
     inputSizeStr = inputSizeStr.substr(0, 3);
     int inputSize = stoi(inputSizeStr);

     //Getting the YOLO specific parameters (anchors & masks)
     std::vector<float> anchors = j["nn_config"]["NN_specific_metadata"]["anchors"];
     std::map<std::string, std::vector<int>> masks = j["nn_config"]["NN_specific_metadata"]["anchor_masks"];

     std::string nnPath(BLOB_PATH);

     // If path to blob specified, use that
     if (argc > 1)
     {
          nnPath = std::string(argv[1]);
     }

     // Print which blob we are using
     printf("Using blob at path: %s\n", nnPath.c_str());

     // Create pipeline
     dai::Pipeline pipeline;

     // Define sources and outputs
     auto camRgb = pipeline.create<dai::node::ColorCamera>();
     auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
     auto monoLeft = pipeline.create<dai::node::MonoCamera>();
     auto monoRight = pipeline.create<dai::node::MonoCamera>();
     auto stereo = pipeline.create<dai::node::StereoDepth>();

     auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
     auto xoutNN = pipeline.create<dai::node::XLinkOut>();
     auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
     auto nnNetworkOut = pipeline.create<dai::node::XLinkOut>();

     xoutRgb->setStreamName("rgb");
     xoutNN->setStreamName("detections");
     xoutDepth->setStreamName("depth");
     nnNetworkOut->setStreamName("nnNetwork");

     // Properties
     camRgb->setPreviewSize(inputSize, inputSize);
     camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
     camRgb->setInterleaved(false);
     camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

     monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
     monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
     monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
     monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

     // setting node configs
     stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
     // Align depth map to the perspective of RGB camera, on which inference is done
     stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
     stereo->setOutputSize(monoLeft->getResolutionWidth(), monoLeft->getResolutionHeight());

     spatialDetectionNetwork->setBlobPath(nnPath);
     spatialDetectionNetwork->setConfidenceThreshold(0.5f);
     spatialDetectionNetwork->input.setBlocking(false);
     spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
     spatialDetectionNetwork->setDepthLowerThreshold(100);
     spatialDetectionNetwork->setDepthUpperThreshold(5000);

     // YOLO specific parameters
     spatialDetectionNetwork->setNumClasses(2);
     spatialDetectionNetwork->setCoordinateSize(4);
     spatialDetectionNetwork->setAnchors(anchors);
     spatialDetectionNetwork->setAnchorMasks(masks);
     spatialDetectionNetwork->setIouThreshold(0.5f);

     // Linking
     monoLeft->out.link(stereo->left);
     monoRight->out.link(stereo->right);

     camRgb->preview.link(spatialDetectionNetwork->input);
     if (syncNN)
     {
          spatialDetectionNetwork->passthrough.link(xoutRgb->input);
     }
     else
     {
          camRgb->preview.link(xoutRgb->input);
     }

     spatialDetectionNetwork->out.link(xoutNN->input);

     stereo->depth.link(spatialDetectionNetwork->inputDepth);
     spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
     spatialDetectionNetwork->outNetwork.link(nnNetworkOut->input);

     // Connect to device and start pipeline
     dai::Device device(pipeline);

     // Output queues will be used to get the rgb frames and nn data from the outputs defined above
     auto previewQueue = device.getOutputQueue("rgb", 4, false);
     auto detectionNNQueue = device.getOutputQueue("detections", 4, false);
     auto depthQueue = device.getOutputQueue("depth", 4, false);
     auto networkQueue = device.getOutputQueue("nnNetwork", 4, false);

     auto startTime = steady_clock::now();
     int counter = 0;
     float fps = 0;
     auto color = cv::Scalar(0, 255, 0);
     bool printOutputLayersOnce = true;

     while (true)
     {
          auto imgFrame = previewQueue->get<dai::ImgFrame>();
          auto inDet = detectionNNQueue->get<dai::SpatialImgDetections>();
          auto depth = depthQueue->get<dai::ImgFrame>();
          auto inNN = networkQueue->get<dai::NNData>();

          if (printOutputLayersOnce && inNN)
          {
               std::cout << "Output layer names: ";
               for (const auto &ten : inNN->getAllLayerNames())
               {
                    std::cout << ten << ", ";
               }
               std::cout << std::endl;
               printOutputLayersOnce = false;
          }

          cv::Mat frame = imgFrame->getCvFrame();
          cv::Mat depthFrame = depth->getFrame(); // depthFrame values are in millimeters

          cv::Mat depthFrameColor;
          cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
          cv::equalizeHist(depthFrameColor, depthFrameColor);
          cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

          counter++;
          auto currentTime = steady_clock::now();
          auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
          if (elapsed > seconds(1))
          {
               fps = counter / elapsed.count();
               counter = 0;
               startTime = currentTime;
          }

          auto detections = inDet->detections;

          for (const auto &detection : detections)
          {
               auto roiData = detection.boundingBoxMapping;
               auto roi = roiData.roi;
               roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
               auto topLeft = roi.topLeft();
               auto bottomRight = roi.bottomRight();
               auto xmin = (int)topLeft.x;
               auto ymin = (int)topLeft.y;
               auto xmax = (int)bottomRight.x;
               auto ymax = (int)bottomRight.y;
               cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);

               int x1 = detection.xmin * frame.cols;
               int y1 = detection.ymin * frame.rows;
               int x2 = detection.xmax * frame.cols;
               int y2 = detection.ymax * frame.rows;

               uint32_t labelIndex = detection.label;
               std::string labelStr = std::to_string(labelIndex);
               if (labelIndex < labelMap.size())
               {
                    labelStr = labelMap[labelIndex];
               }
               cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
               std::stringstream confStr;
               confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
               cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

               std::stringstream depthX;
               depthX << "X: " << (int)detection.spatialCoordinates.x << " mm";
               cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
               std::stringstream depthY;
               depthY << "Y: " << (int)detection.spatialCoordinates.y << " mm";
               cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
               std::stringstream depthZ;
               depthZ << "Z: " << (int)detection.spatialCoordinates.z << " mm";
               cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

               cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);

               std::cout << "New cone:" << std::endl << "X:" << (int)detection.spatialCoordinates.x << std::endl << "Y:" << (int)detection.spatialCoordinates.y << std::endl << "Z:" << (int)detection.spatialCoordinates.z << std::endl << "Lable:" << labelStr << std::endl << std::endl;
          }

          std::stringstream fpsStr;
          fpsStr << std::fixed << std::setprecision(2) << fps;
          cv::putText(frame, fpsStr.str(), cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

          cv::imshow("depth", depthFrameColor);
          cv::imshow("rgb", frame);

          int key = cv::waitKey(1);
          if (key == 'q' || key == 'Q')
          {
               return 0;
          }
     }
     return 0;
}
