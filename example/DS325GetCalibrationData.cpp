/**
 * @file DS325GetCalibrationData.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 18, 2014
 */

#include <stdio.h>
#include <vector>
#include <exception>
#include <string>
#include <ctime>
#include <DepthSense.hxx>
#include <opencv2/opencv.hpp>
#include <gflags/gflags.h>

using namespace DepthSense;
using namespace std;
using namespace google;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;
AudioNode g_anode;

uint32_t g_aFrames = 0;
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;

int imageNum = 0;

cv::Mat g_depth,g_color;

#define MAX_DEPTH 1000
#define MIN_DEPTH 0

// definition for gflags
DEFINE_string(folder, "calibData", "calibration data folder name");
DEFINE_string(depth, "depth_", "depth file name");
DEFINE_string(color, "color_", "color file name");
DEFINE_string(type, ".png", "file type");

/*----------------------------------------------------------------------------*/
// New color sample event handler
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data){
  printf("colorflame %u: %d\n",g_cFrames,data.colorMap.size());
  int key = 0;
  memcpy(g_color.data, data.colorMap, data.colorMap.size());

  cv::Mat scaledDepth(g_depth.rows * 2, g_depth.cols * 2, CV_16UC1);

  cv::Mat maxDist = cv::Mat::ones(g_color.rows , g_color.cols , CV_16UC1) * MAX_DEPTH;
  cv::Mat minDist = cv::Mat::ones(g_color.rows , g_color.cols , CV_16UC1) * MIN_DEPTH;
  cv::resize(g_depth, scaledDepth, cv::Size(), 2.0,2.0);
  cv::min(scaledDepth, maxDist, scaledDepth);

  cv::Size patternSize = cv::Size(9, 6);
  std::vector<cv::Point2f> imagePoints[2];

  scaledDepth -= minDist;
  scaledDepth.convertTo(scaledDepth, CV_8UC1, 255.0 / (MAX_DEPTH - MIN_DEPTH));



  if( cv::findChessboardCorners(g_color, patternSize, imagePoints[0],
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE) &&
      cv::findChessboardCorners(scaledDepth, patternSize, imagePoints[1],
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE)) {

    std::cout << "found" << std::endl;
    key = cv::waitKey(1);
    if(key == 't'){

      stringstream ss_c, ss_d;
      ss_c << FLAGS_folder << "/" << FLAGS_color << imageNum << FLAGS_type;
      ss_d << FLAGS_folder << "/" << FLAGS_depth << imageNum << FLAGS_type;

      cv::imwrite(ss_c.str(), g_color);
      cv::imwrite(ss_d.str(), g_depth);

      imageNum++;

      cout << "image " << imageNum << " saved" << endl;

    }

    cv::rectangle(g_color, cv::Point(5,5), cv::Point(635, 475), cv::Scalar(255,0,0), 3);
  }else
    cv::rectangle(g_color, cv::Point(5,5), cv::Point(635, 475), cv::Scalar(0,0,255), 3);


  cv::imshow("color", g_color);
  cv::imshow("depth", scaledDepth);

  key = cv::waitKey(1);
  if(key == 'q'){
    g_context.quit();
  }
  g_cFrames++;
}

/*----------------------------------------------------------------------------*/
// New depth sample event handler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
  printf("depthflame %u: %d\n",g_dFrames,data.vertices.size());
  g_dFrames++;
  memcpy(g_depth.data, data.confidenceMap, data.confidenceMap.size()*2);
}

/*----------------------------------------------------------------------------*/
void configureDepthNode()
{
  g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);

  DepthNode::Configuration config = g_dnode.getConfiguration();
  config.frameFormat = FRAME_FORMAT_QVGA;
  config.framerate = 30;
  config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
  config.saturation = true;

  g_dnode.setEnableVertices(true);
  //g_dnode.setEnableDepthMap(true);
  g_dnode.setEnableConfidenceMap(true);

  try
    {
      g_context.requestControl(g_dnode,0);

      g_dnode.setConfiguration(config);
    }
  catch (ArgumentException& e)
    {
      printf("Argument Exception: %s\n",e.what());
    }
  catch (UnauthorizedAccessException& e)
    {
      printf("Unauthorized Access Exception: %s\n",e.what());
    }
  catch (IOException& e)
    {
      printf("IO Exception: %s\n",e.what());
    }
  catch (InvalidOperationException& e)
    {
      printf("Invalid Operation Exception: %s\n",e.what());
    }
  catch (ConfigurationException& e)
    {
      printf("Configuration Exception: %s\n",e.what());
    }
  catch (StreamingException& e)
    {
      printf("Streaming Exception: %s\n",e.what());
    }
  catch (TimeoutException&)
    {
      printf("TimeoutException\n");
    }

}

/*----------------------------------------------------------------------------*/
void configureColorNode()
{
  // connect new color sample handler
  g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

  ColorNode::Configuration config = g_cnode.getConfiguration();
  //config.frameFormat = FRAME_FORMAT_WXGA_H;
  config.frameFormat = FRAME_FORMAT_VGA;
  config.compression = COMPRESSION_TYPE_MJPEG;
  config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
  config.framerate = 30;

  g_cnode.setEnableColorMap(true);

  try
    {
      g_context.requestControl(g_cnode,0);

      g_cnode.setConfiguration(config);
    }
  catch (ArgumentException& e)
    {
      printf("Argument Exception: %s\n",e.what());
    }
  catch (UnauthorizedAccessException& e)
    {
      printf("Unauthorized Access Exception: %s\n",e.what());
    }
  catch (IOException& e)
    {
      printf("IO Exception: %s\n",e.what());
    }
  catch (InvalidOperationException& e)
    {
      printf("Invalid Operation Exception: %s\n",e.what());
    }
  catch (ConfigurationException& e)
    {
      printf("Configuration Exception: %s\n",e.what());
    }
  catch (StreamingException& e)
    {
      printf("Streaming Exception: %s\n",e.what());
    }
  catch (TimeoutException&)
    {
      printf("TimeoutException\n");
    }
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
  if ((node.is<DepthNode>())&&(!g_dnode.isSet()))
    {
      g_dnode = node.as<DepthNode>();
      configureDepthNode();
      g_context.registerNode(node);
    }

  if ((node.is<ColorNode>())&&(!g_cnode.isSet()))
    {
      g_cnode = node.as<ColorNode>();
      configureColorNode();
      g_context.registerNode(node);
    }

  cv::namedWindow("color");
  cv::namedWindow("depth");
}

/*----------------------------------------------------------------------------*/
void onNodeConnected(Device device, Device::NodeAddedData data)
{
  configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
  // if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode))
  //     g_anode.unset();
  if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
    g_cnode.unset();
  if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
    g_dnode.unset();
  printf("Node disconnected\n");
}

/*----------------------------------------------------------------------------*/
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
  if (!g_bDeviceFound)
    {
      data.device.nodeAddedEvent().connect(&onNodeConnected);
      data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
      g_bDeviceFound = true;
    }

  cv::destroyAllWindows();
}

/*----------------------------------------------------------------------------*/
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
  g_bDeviceFound = false;
  printf("Device disconnected\n");
}

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
  string usage("This Program get calibration data. \n Would you kindly hit 't' key if rectangle turns blue.");
  SetUsageMessage(usage);
  SetVersionString("1.0");

  ParseCommandLineFlags(&argc, &argv, true);


  g_context = Context::create("localhost");

  g_context.deviceAddedEvent().connect(&onDeviceConnected);
  g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

  // Get the list of currently connected devices
  vector<Device> da = g_context.getDevices();

  // init image
  //g_color = cv::Mat(720, 1280, CV_8UC3);
  g_color = cv::Mat(480, 640, CV_8UC3);
  g_depth = cv::Mat(240, 320, CV_16UC1);

  //create tree directory
  string execstr = "mkdir -p ";
  execstr += FLAGS_folder;
  system( execstr.c_str() );

  // We are only interested in the first device
  if (da.size() >= 1)
    {
      g_bDeviceFound = true;

      da[0].nodeAddedEvent().connect(&onNodeConnected);
      da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

      vector<Node> na = da[0].getNodes();

      printf("Found %u nodes\n",(uint)na.size());

      for (int n = 0; n < (int)na.size();n++)
    configureNode(na[n]);
    }



  g_context.startNodes();

  g_context.run();

  g_context.stopNodes();

  if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
  if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);

  printf("close nodes");

  if (g_pProjHelper)
    delete g_pProjHelper;

  return 0;
}


