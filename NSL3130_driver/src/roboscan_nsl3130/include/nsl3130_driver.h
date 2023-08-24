#ifndef __NSL3130_DRIVER_H__
#define __NSL3130_DRIVER_H__

#include "nsl3130_image.h"
#include "communication.h"
#include "cartesian_transform.hpp"

#include <vector>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>

#include "communication/nsl3130_image.h"


struct Settings{

  int frameRate;
  bool startStream;
  bool triggerSingleShot;
  bool runVideo;
  bool updateParam;
  int integrationTimeTOF1;
  int integrationTimeTOF2;
  int integrationTimeTOF3;
  int integrationTimeGray;

  int imageType;
  int hdrMode;
  int modFrequency;
  int modChannel;

  bool medianFilter;
  bool averageFilter;
  double kalmanFactor;
  int kalmanThreshold;
  int edgeThreshold;
  int interferenceDetectionLimit;
  bool interferenceDetectionUseLastValue;

  int minAmplitude;
  int minDistance;
  int maxDistance;  
  int lensCenterOffsetX;
  int lensCenterOffsetY;
  bool enableCartesian;
  bool enableImages;
  bool enablePointCloud;  
  bool enableTemperature;

  int roi_leftX;
  int roi_topY;
  int roi_rightX;
  int roi_bottomY;
  int lenstype;

  bool cvShow;
  bool changedCvShow;

};



class Nsl3130Driver : public rclcpp::Node 
{

public:

    Nsl3130Driver();
    ~Nsl3130Driver();
    void update();   
    void initCommunication();
    void closeCommunication();

private:

    double angle;
    bool lastSingleShot;
    bool lastStreaming;
    double framePeriod;
	rclcpp::Clock _ros_clock;
    std::chrono::system_clock::time_point timeLast;
    std::chrono::system_clock::time_point one_second;
	int frameCnt;
	int frameAddCnt;
    uint frameSeq;    

	static Settings settings;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher1;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2Publisher;

    uint imageSize8;
    uint imageSize16_1;
    uint imageSize16_2;
    std::string strFrameID;
    sensor_msgs::msg::Image img8;
    sensor_msgs::msg::Image img16_1;
    sensor_msgs::msg::Image img16_2;
//    sensor_msgs::Temperature msgTemperature;
    com_lib::Communication communication;

    int lensTableSize;
    int numCols;
    int numRows;
    double lensAngle[101];
    double rp[101];
    double xUA[320][240];
    double yUA[320][240];
    double zUA[320][240];
    double sensorPointSizeMM;
    int oldLensCenterOffsetX;
    int oldLensCenterOffsetY;

	void setWinName();
    void updateData();
    void setParameters();
    void updateTemperature(double temperature);
    void updateGrayscaleFrame(std::shared_ptr<com_lib::Nsl3130Image> image);
    void updateDistanceFrame(std::shared_ptr<com_lib::Nsl3130Image> image);
    void updateDistanceAmplitudeFrame(std::shared_ptr<com_lib::Nsl3130Image> image);
	void publisherPointCloud(std::shared_ptr<com_lib::Nsl3130Image> image);

	void parameterInit();
	void getMouseEvent( int &mouse_xpos, int &mouse_ypos );
	double interpolate( double x, double x0, double y0, double x1, double y1);
	void createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);
	int setDistanceColor(cv::Mat &imageLidar, int x, int y, int value );
	void setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value );
	void setGrayScaledColor(cv::Mat &imageLidar, int x, int y, int value, double end_range );
	cv::Mat addDistanceInfo(cv::Mat distMat, std::shared_ptr<com_lib::Nsl3130Image> image);
    void thread_callback();
    boost::scoped_ptr<boost::thread> publisherThread;
    bool runThread;
	OnSetParametersCallbackHandle::SharedPtr callback_handle_;
	rcl_interfaces::msg::SetParametersResult parametersCallback( const std::vector<rclcpp::Parameter> &parameters);
	std::vector<uint16_t> dist2BData;
	std::vector<uint16_t> ampl2BData;
	std::vector<uint16_t> gray2BData;

    CartesianTransform cartesian;
	int mouseXpos, mouseYpos;
	std::vector<cv::Vec3b> colorVector;
	char winName[100];

};


#endif // __NSL3130_DRIVER_H__
