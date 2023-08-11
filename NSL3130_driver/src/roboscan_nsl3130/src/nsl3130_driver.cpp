
#include "nsl3130_driver.h"
#include <sensor_msgs/image_encodings.hpp>
//#include <duration.hpp>

using namespace com_lib;
using namespace std;


#define WIN_NAME 					"LiDAR"
#define NSL3130_NUM_COLORS     		30000
#define NSL3130_PIXEL_CODE_OFFSET	48000
//Special codes for pixels without valid data
#define NSL3130_LIMIT_FOR_VALID_DATA 	(16000 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_LOW_AMPLITUDE			(16001 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_ADC_OVERFLOW			(16002 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_SATURATION				(16003 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_BAD_PIXEL 				(16004 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_LOW_DCS					(16005 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_INTERFERENCE			(16007 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_EDGE_DETECTED			(16008 + NSL3130_PIXEL_CODE_OFFSET)


static int maxAmplitudeValue = 2897;
static const int indexAmplitudeFactorColor = NSL3130_NUM_COLORS / maxAmplitudeValue;

static Settings settings;
static Settings settings_callback;
Settings *Nsl3130Driver::gSettings;


std::atomic<int> x_start = -1, y_start = -1;

static void callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	std::ignore = flags;
	std::ignore = user_data;
	
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		x_start = x;
		y_start = y;
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
	}
}

void Nsl3130Driver::setWinName()
{
	bool changedCvShow = settings.changedCvShow;
	settings.changedCvShow = false;

	printf("changedCvShow = %d cvShow = %d\n", changedCvShow, settings.cvShow);
	
	if( changedCvShow ){
		cv::destroyAllWindows();
	}
	
	if( settings.cvShow == false || changedCvShow == false ) return;
	
	if( settings.imageType == 0 ){
		sprintf(winName,"%s(Gray)", WIN_NAME);
	}
	else if( settings.imageType == 1 ){
		sprintf(winName,"%s(Dist)", WIN_NAME);
	}
	else {//if( settings.imageType == 2 ){
		sprintf(winName,"%s(Dist/Ampl)", WIN_NAME);
	}
	
	cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
	cv::setWindowProperty(winName, cv::WND_PROP_TOPMOST, 1);	
	cv::setMouseCallback(winName, callback_mouse_click, NULL);

	printf("namedWindow()~~~~~~~ winName = %s\n", winName);
}


Nsl3130Driver::Nsl3130Driver() : Node("roboscan_publish_node"),
	_ros_clock(RCL_ROS_TIME)
{
	printf("Nsl3130Driver()\n");	

	int numSteps = NSL3130_NUM_COLORS;
	unsigned char red, green, blue;


	for(int i=0;  i< numSteps; i++)
	{
	  createColorMapPixel(numSteps, i, red, green, blue);
	  colorVector.push_back(cv::Vec3b(blue, green, red));
	}

	frameCnt = 0;
	frameAddCnt = 0;

    numCols = 320;
    numRows = 240;
    frameSeq = 0;
    imageSize8 = 0;
    imageSize16_1 = 0;
    imageSize16_2 = 0;
    sensorPointSizeMM = 0.02; //sensor pixel size mm
    lastSingleShot = false;
    lastStreaming = false;
    strFrameID = "roboscan_frame";

	gSettings = &settings;
    gSettings->runVideo = false;
	gSettings->imageType = Nsl3130Image::ImageType_e::DISTANCE_AMPLITUDE;
	gSettings->lenstype = 1;
	gSettings->frameRate = 20; //fps
	gSettings->lensCenterOffsetX = 0;
	gSettings->lensCenterOffsetY = 0;
	gSettings->hdrMode = 0;
	gSettings->integrationTimeTOF1 = 1500;
	gSettings->integrationTimeTOF2 = 500;
	gSettings->integrationTimeTOF3 = 50;
	gSettings->integrationTimeGray = 100;
	gSettings->modFrequency = 1;
	gSettings->modChannel = 0;

	gSettings->medianFilter = false;
	gSettings->averageFilter = false;
	gSettings->kalmanFactor = 0;
	gSettings->kalmanThreshold = 300;
	gSettings->edgeThreshold = 0;
	gSettings->interferenceDetectionLimit = 0;
	gSettings->interferenceDetectionUseLastValue = 0;

	gSettings->minDistance = 30;
	gSettings->maxDistance = 12500;

	gSettings->minAmplitude = 100;
	gSettings->startStream = false;
	
    gSettings->enableCartesian   = true;
    gSettings->enableTemperature = false;
    gSettings->enableImages      = true;
    gSettings->enablePointCloud  = true;

    gSettings->roi_leftX   = 0;
    gSettings->roi_rightX  = 319;
	gSettings->roi_topY = 0;
	gSettings->roi_bottomY = 239;

	gSettings->runVideo = false;
	gSettings->triggerSingleShot = false;
	gSettings->updateParam = true;
	gSettings->cvShow = false;
	gSettings->changedCvShow = false;

	parameterInit();

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    imagePublisher1 = create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile); 
    imagePublisher2 = create_publisher<sensor_msgs::msg::Image>("roboscanAmplitude", qos_profile); 
    pointCloud2Publisher = create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile); 
    cameraInfoPublisher  = create_publisher<sensor_msgs::msg::CameraInfo>("roboscanCameraInfo", qos_profile); 
    imageHeaderPublisher = create_publisher<std_msgs::msg::Int32MultiArray>("roboscanImageHeader", qos_profile); 

    initCommunication();

    communication.sigReceivedGrayscale.connect(boost::bind(&Nsl3130Driver::updateGrayscaleFrame, this, _1));
    communication.sigReceivedDistance.connect(boost::bind(&Nsl3130Driver::updateDistanceFrame, this, _1));
    communication.sigReceivedDistanceAmplitude.connect(boost::bind(&Nsl3130Driver::updateDistanceAmplitudeFrame, this, _1));    

    gSettings->updateParam = true;
    timeLast = std::chrono::system_clock::now();

    setParameters();
	
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&Nsl3130Driver::parametersCallback, this, std::placeholders::_1));
    
    runThread = true;
    publisherThread.reset(new boost::thread(boost::bind(&Nsl3130Driver::thread_callback, this)));
}

Nsl3130Driver::~Nsl3130Driver()
{
    printf("Nsl3130Driver::~Nsl3130Driver()\n");
    runThread = false;
    publisherThread->join();

//    communication.stopStream();
//    sleep(1);
    communication.close();    
    printf("END ~Nsl3130Driver()\n");
}

void Nsl3130Driver::thread_callback()
{
	printf("start thread_callback\n");
    while(runThread)
    {
        update();
    }

	cv::destroyAllWindows();
	printf("end thread_callback\n");
}

void Nsl3130Driver::parameterInit()
{
	rclcpp::Parameter pImageType("A. imageType", settings.imageType);
	rclcpp::Parameter pModeFrequency("B. modFrequency", settings.modFrequency);
	rclcpp::Parameter pStartStream("C. startStream", settings.startStream);
	rclcpp::Parameter pHdr("D. Hdr", settings.hdrMode);
	rclcpp::Parameter pIntegrationTime0("E. integrationTime0", settings.integrationTimeTOF1);
	rclcpp::Parameter pIntegrationTime1("F. integrationTime1", settings.integrationTimeTOF2);
	rclcpp::Parameter pIntegrationTime2("G. integrationTime2", settings.integrationTimeTOF3);
	rclcpp::Parameter pIntegrationTimeGray("H. integrationTimeGray", settings.integrationTimeGray);
	rclcpp::Parameter pTemporalFilterFactor("I. temporalFilterFactor", settings.kalmanFactor);
	rclcpp::Parameter pTemporalFilterThreshold("J. temporalFilterThreshold", settings.kalmanThreshold);
	rclcpp::Parameter pMedianFilter("K. medianFilter", settings.medianFilter);
	rclcpp::Parameter pAverageFilter("L. averageFilter", settings.averageFilter);
	rclcpp::Parameter pEdgeThreshold("M. edgeThreshold", settings.edgeThreshold);
	rclcpp::Parameter pMinAmplitude("N. minAmplitude", settings.minAmplitude);
	rclcpp::Parameter pMinDistance("O. minDistance", settings.minDistance);
	rclcpp::Parameter pMaxDistance("P. maxDistance", settings.maxDistance);
	rclcpp::Parameter pRoiLeftX("Q. roiLeftX", settings.roi_leftX);
	rclcpp::Parameter pRoiTopY("R. roiTopY", settings.roi_topY);
	rclcpp::Parameter pRoiRightX("S. roiRightX", settings.roi_rightX);
	rclcpp::Parameter pRoiBottomY("T. roiBottomY", settings.roi_bottomY);
	rclcpp::Parameter pLensType("U. lensType", settings.lenstype);
	rclcpp::Parameter pTriggerSingleShot("V. triggerSingleShot", settings.triggerSingleShot);
	rclcpp::Parameter pCvshow("W. cvShow", settings.cvShow);
	rclcpp::Parameter pFps("X. frameRate", settings.frameRate);
	
	this->declare_parameter<int>("A. imageType", settings.imageType);
	this->declare_parameter<int>("B. modFrequency", settings.modFrequency);		
	this->declare_parameter<bool>("C. startStream", settings.startStream);	
	this->declare_parameter<int>("D. Hdr", settings.hdrMode);	
	this->declare_parameter<int>("E. integrationTime0", settings.integrationTimeTOF1);
	this->declare_parameter<int>("F. integrationTime1", settings.integrationTimeTOF2);	
	this->declare_parameter<int>("G. integrationTime2", settings.integrationTimeTOF3);
	this->declare_parameter<int>("H. integrationTimeGray", settings.integrationTimeGray);
	this->declare_parameter<double>("I. temporalFilterFactor", settings.kalmanFactor);
	this->declare_parameter<int>("J. temporalFilterThreshold", settings.kalmanThreshold);
	this->declare_parameter<bool>("K. medianFilter", settings.medianFilter);
	this->declare_parameter<bool>("L. averageFilter", settings.averageFilter);
	this->declare_parameter<int>("M. edgeThreshold", settings.edgeThreshold);
	this->declare_parameter<int>("N. minAmplitude", settings.minAmplitude);
	this->declare_parameter<int>("O. minDistance", settings.minDistance);
	this->declare_parameter<int>("P. maxDistance", settings.maxDistance);
	this->declare_parameter<int>("Q. roiLeftX", settings.roi_leftX);
	this->declare_parameter<int>("R. roiTopY", settings.roi_topY);
	this->declare_parameter<int>("S. roiRightX", settings.roi_rightX);
	this->declare_parameter<int>("T. roiBottomY", settings.roi_bottomY);
	this->declare_parameter<int>("U. lensType", settings.lenstype);
	this->declare_parameter<bool>("V. triggerSingleShot", settings.triggerSingleShot);
	this->declare_parameter<bool>("W. cvShow", settings.cvShow);
	this->declare_parameter<int>("X. frameRate", settings.frameRate);
	

	this->set_parameter(pImageType);
	this->set_parameter(pModeFrequency);	
	this->set_parameter(pStartStream);
	this->set_parameter(pHdr);
	this->set_parameter(pIntegrationTime0);
	this->set_parameter(pIntegrationTime1);
	this->set_parameter(pIntegrationTime2);
	this->set_parameter(pIntegrationTimeGray);
	this->set_parameter(pTemporalFilterFactor);
	this->set_parameter(pTemporalFilterThreshold);
	this->set_parameter(pMedianFilter);
	this->set_parameter(pAverageFilter);
	this->set_parameter(pEdgeThreshold);
	this->set_parameter(pMinAmplitude);
	this->set_parameter(pMinDistance);
	this->set_parameter(pMaxDistance);
	this->set_parameter(pRoiLeftX);
	this->set_parameter(pRoiTopY);
	this->set_parameter(pRoiRightX);
	this->set_parameter(pRoiBottomY);
	this->set_parameter(pLensType);
	this->set_parameter(pTriggerSingleShot);
	this->set_parameter(pCvshow);
	this->set_parameter(pFps);
	


	memcpy(&settings_callback, &settings, sizeof(settings_callback));
}


rcl_interfaces::msg::SetParametersResult Nsl3130Driver::parametersCallback( const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    
    for (const auto &param: parameters)
	{
		if (param.get_name() == "A. imageType")
		{
			int imgType = param.as_int();
			if( imgType < 0 ) imgType = 0;
			if( imgType > 2 ) imgType = 2;
			
			if( settings_callback.imageType != imgType ){
				settings_callback.imageType = imgType;
				settings_callback.changedCvShow = true;
			}
		}
		else if (param.get_name() == "B. modFrequency")
		{
			settings_callback.modFrequency = param.as_int();
			if( settings_callback.modFrequency < 0 ) settings_callback.modFrequency = 0;
			if( settings_callback.modFrequency > 3 ) settings_callback.modFrequency = 3;
		}
		else if (param.get_name() == "C. startStream")
		{
			settings_callback.startStream = param.as_bool();
		}
		else if (param.get_name() == "D. Hdr")
		{
			settings_callback.hdrMode = param.as_int();
			if( settings_callback.hdrMode < 0 ) settings_callback.hdrMode = 0;
			if( settings_callback.hdrMode > 2 ) settings_callback.hdrMode = 2;
		}
		else if (param.get_name() == "E. integrationTime0")
		{
			settings_callback.integrationTimeTOF1 = param.as_int();
			if( settings_callback.integrationTimeTOF1 < 0 ) settings_callback.integrationTimeTOF1 = 0;
			if( settings_callback.integrationTimeTOF1 > 4000 ) settings_callback.integrationTimeTOF1 = 4000;
		}
		else if (param.get_name() == "F. integrationTime1")
		{
			settings_callback.integrationTimeTOF2 = param.as_int();
			if( settings_callback.integrationTimeTOF2 < 0 ) settings_callback.integrationTimeTOF2 = 0;
			if( settings_callback.integrationTimeTOF2 > 4000 ) settings_callback.integrationTimeTOF2 = 4000;
		}
		else if (param.get_name() == "G. integrationTime2")
		{
			settings_callback.integrationTimeTOF3 = param.as_int();
			if( settings_callback.integrationTimeTOF3 < 0 ) settings_callback.integrationTimeTOF3 = 0;
			if( settings_callback.integrationTimeTOF3 > 4000 ) settings_callback.integrationTimeTOF3 = 4000;
		}
		else if (param.get_name() == "H. integrationTimeGray")
		{
			settings_callback.integrationTimeGray = param.as_int();
			if( settings_callback.integrationTimeGray < 0 ) settings_callback.integrationTimeGray = 0;
			if( settings_callback.integrationTimeGray > 40000 ) settings_callback.integrationTimeGray = 40000;
		}
		else if (param.get_name() == "I. temporalFilterFactor")
		{
			settings_callback.kalmanFactor = param.as_double();
		}
		else if (param.get_name() == "J. temporalFilterThreshold")
		{
			settings_callback.kalmanThreshold = param.as_int();
		}
		else if (param.get_name() == "K. medianFilter")
		{
			settings_callback.medianFilter = param.as_bool();
		}
		else if (param.get_name() == "L. averageFilter")
		{
			settings.averageFilter = param.as_bool();
		}
		else if (param.get_name() == "M. edgeThreshold")
		{
			settings_callback.edgeThreshold = param.as_int();
		}
		else if (param.get_name() == "N. minAmplitude")
		{
			settings_callback.minAmplitude = param.as_int();
		}
		else if (param.get_name() == "O. minDistance")
		{
			settings_callback.minDistance = param.as_int();
		}
		else if (param.get_name() == "P. maxDistance")
		{
			settings_callback.maxDistance = param.as_int();
		}
		else if (param.get_name() == "Q. roiLeftX")
		{
			settings_callback.roi_leftX= param.as_int();
		}
		else if (param.get_name() == "R. roiTopY")
		{
			settings_callback.roi_topY= param.as_int();
		}
		else if (param.get_name() == "S. roiRightX")
		{
			settings_callback.roi_rightX= param.as_int();
		}
		else if (param.get_name() == "T. roiBottomY")
		{
			settings_callback.roi_bottomY= param.as_int();
		}
		else if (param.get_name() == "U. lensType")
		{
			settings_callback.lenstype= param.as_int();
		}
		else if (param.get_name() == "V. triggerSingleShot")
		{
			settings_callback.triggerSingleShot = param.as_bool();
		}
		else if (param.get_name() == "W. cvShow")
		{
			bool showCv = param.as_bool();
			if( settings_callback.cvShow != showCv ){
				settings_callback.cvShow = showCv;
				settings_callback.changedCvShow = true;
			}
		}
		else if( param.get_name() == "X. frameRate")
		{
			settings_callback.frameRate = param.as_int();
			if( settings_callback.frameRate == 0 ) settings_callback.frameRate = 1;
		}
	}
    
	printf("parametersCallback() stream = %d type =%d\n", settings_callback.startStream, settings_callback.imageType);
	
    settings.updateParam = true;
    return result;
}


void Nsl3130Driver::closeCommunication()
{
    //ROS_DEBUG("Nsl3130Driver::closeCommunication()"); //TODo remove
    communication.close();
}


void Nsl3130Driver::update()
{
    std::chrono::system_clock::time_point timeNow = std::chrono::system_clock::now();
	std::chrono::milliseconds elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - one_second);

    if(elapsed_time.count() >= 1000 ){
		one_second = timeNow;
		frameCnt = frameAddCnt;
		frameAddCnt = 0;
	}

    if(gSettings->runVideo && !gSettings->updateParam){
        updateData(); //streaming

    }else if(gSettings->updateParam){
        setParameters(); //update parameters

        if(gSettings->triggerSingleShot && gSettings->triggerSingleShot != lastSingleShot)
            updateData(); //trigger single shot

        lastSingleShot = gSettings->triggerSingleShot;
    }
}

void Nsl3130Driver::setParameters()
{
    if(gSettings->updateParam)
    {
		memcpy(&settings, &settings_callback, sizeof(settings_callback));
        gSettings->updateParam = false;

        printf("update parameters hdr = %d\n", gSettings->hdrMode);

        framePeriod = 1.0 / gSettings->frameRate * 1000.0f; // msec

		int modFrequency = gSettings->modFrequency == 0 ? 1 : gSettings->modFrequency == 1 ? 0 : gSettings->modFrequency > 3 ? 3 : gSettings->modFrequency;
		int modChannel = gSettings->modChannel;

        communication.setHDRMode(gSettings->hdrMode);
        communication.setIntegrationTime3d(gSettings->integrationTimeTOF1, gSettings->integrationTimeTOF2, gSettings->integrationTimeTOF3, gSettings->integrationTimeGray);        
        communication.setMinimalAmplitude(gSettings->minAmplitude);
        communication.setModulationFrequency(modFrequency, modChannel);
        communication.setFilter(gSettings->medianFilter, gSettings->averageFilter, 1000.0 * gSettings->kalmanFactor, gSettings->kalmanThreshold,
                                gSettings->edgeThreshold, gSettings->interferenceDetectionLimit, gSettings->interferenceDetectionUseLastValue);
        communication.setRoi(gSettings->roi_leftX, gSettings->roi_topY, gSettings->roi_rightX, gSettings->roi_bottomY);

        cartesian.initLensTransform(sensorPointSizeMM, numCols, numRows, gSettings->lensCenterOffsetX, gSettings->lensCenterOffsetY, gSettings->lenstype);
		oldLensCenterOffsetX = gSettings->lensCenterOffsetX;
		oldLensCenterOffsetY = gSettings->lensCenterOffsetY;	

	    if(gSettings->startStream)
	        gSettings->runVideo = true;
		else
			gSettings->runVideo = false;

		setWinName();

    } //END if(gSettings->updateParam)
}


void Nsl3130Driver::updateData()
{
    std::chrono::system_clock::time_point timeNow = std::chrono::system_clock::now();
	std::chrono::milliseconds elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - timeLast);

    if(elapsed_time.count() >= framePeriod )
	{
//		printf("milli = %ld period = %.1f\n", elapsed_time.count(), framePeriod);
        timeLast = timeNow;

//        printf("Type = %d\n", gSettings->imageType);

        switch(gSettings->imageType)
        {
            case Nsl3130Image::ImageType_e::GRAYSCALE:
                  communication.getGrayscale(com_const::Acquisition::VALUE_AUTO_REPEAT_MEASUREMENT);
                  break;
            case Nsl3130Image::ImageType_e::DISTANCE:
                  communication.getDistance(com_const::Acquisition::VALUE_AUTO_REPEAT_MEASUREMENT);
                  break;
            case Nsl3130Image::ImageType_e::DISTANCE_AMPLITUDE:
			default:
                  communication.getDistanceAmplitude(com_const::Acquisition::VALUE_AUTO_REPEAT_MEASUREMENT);
                  break;
        }

    } //end if elapsed_time

}

void Nsl3130Driver::initCommunication()
{
	string port_name = "/dev/ttyLidar";
	communication.open(port_name); //open serial port  
	usleep(500000);

	unsigned int minor, major;
	communication.getFirmwareRelease(major, minor);
	printf("Firmware release:  major= %d  minor= %d\n", major, minor);

	uint16_t chipID, waferID;
	communication.getChipInformation(chipID, waferID);
	printf("Chip ID= %d   Wafer ID= %d\n", chipID, waferID);
}

void Nsl3130Driver::getMouseEvent( int &mouse_xpos, int &mouse_ypos )
{
	mouse_xpos = x_start;
	mouse_ypos = y_start;
}

double Nsl3130Driver::interpolate( double x, double x0, double y0, double x1, double y1){

    if( x1 == x0 ){
        return y0;
    } else {
        return ((x-x0)*(y1-y0)/(x1-x0) + y0);
    }

}

void Nsl3130Driver::createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue)
{
    double k = 1;
    double BIT0 = -0.125 * k - 0.25;
    double BIT1 = BIT0 + 0.25 * k;
    double BIT2 = BIT1 + 0.25 * k;
    double BIT3 = BIT2 + 0.25 * k;

    double G0 = BIT1;
    double G1 = G0 + 0.25 * k;
    double G2 = G1 + 0.25 * k;
    double G3 = G2 + 0.25 * k + 0.125;

    double R0 = BIT2;
    double R1 = R0 + 0.25 * k;
    double R2 = R1 + 0.25 * k;
    double R3 = R2 + 0.25 * k + 0.25;

    double i = (double)indx/(double)numSteps - 0.25 * k;

    if( i>= R0 && i < R1 ){
        red = interpolate(i, R0, 0, R1, 255);
    } else if((i >= R1) && (i < R2)){
        red = 255;
    } else if((i >= R2) && (i < R3)) {
        red = interpolate(i, R2, 255, R3, 0);
    } else {
        red = 0;
    }

    if( i>= G0 && i < G1 ){
        green = interpolate(i, G0, 0, G1, 255);
    } else if((i>=G1)&&(i<G2)){
        green = 255;
    } else if((i >= G2)&&(i < G3)){
        green = interpolate(i, G2, 255, G3, 0);
    } else {
        green = 0;
    }


    if( i>= BIT0 && i < BIT1 ){
        blue = interpolate(i, BIT0, 0, BIT1, 255);
    } else if((i >= BIT1)&&(i < BIT2)){
        blue = 255;
    } else if((i >= BIT2)&&(i < BIT3)) {
        blue = interpolate(i, BIT2, 255, BIT3, 0);
    } else{
        blue = 0;
    }

}


int Nsl3130Driver::setDistanceColor(cv::Mat &imageLidar, int x, int y, int value )
{
	if( value == NSL3130_LOW_AMPLITUDE )
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_SATURATION)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 128;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 255; 
	}
	else if (value == NSL3130_ADC_OVERFLOW)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 255;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 14;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 169; 
	}
	else if(value == NSL3130_INTERFERENCE)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_EDGE_DETECTED)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_BAD_PIXEL)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
//		imageLidar.at<Vec3b>(y, x)[0] = 255;
//		imageLidar.at<Vec3b>(y, x)[1] = 255;
//		imageLidar.at<Vec3b>(y, x)[2] = 255; 
		imageLidar.at<cv::Vec3b>(y, x) = colorVector.at(colorVector.size()-1);
	}
	else if (value < 0)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if (value > settings.maxDistance)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = colorVector.size() - (value*(NSL3130_NUM_COLORS / settings.maxDistance));
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = colorVector.size()-1;
		}
		else if( index > (int)colorVector.size() ){
			index = 0;
		}
		
		imageLidar.at<cv::Vec3b>(y, x) = colorVector.at(index);
	}

	return value;
}

void Nsl3130Driver::setGrayScaledColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
{
	if (value == NSL3130_SATURATION)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 128;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 255; 
	}
	else if (value == NSL3130_ADC_OVERFLOW)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 255;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 14;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 169; 
	}
	else if (value < 0)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if (value > end_range)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 255;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 255;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 255; 
	}
	else{
		int color = value * (255.0 /end_range);
		imageLidar.at<cv::Vec3b>(y, x)[0] = color;
		imageLidar.at<cv::Vec3b>(y, x)[1] = color;
		imageLidar.at<cv::Vec3b>(y, x)[2] = color; 
	}

}



void Nsl3130Driver::setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value )
{
	if( value == NSL3130_LOW_AMPLITUDE )
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_SATURATION)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 128;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 255; 
	}
	else if (value == NSL3130_ADC_OVERFLOW)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 255;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 14;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 169; 
	}
	else if(value == NSL3130_INTERFERENCE)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_EDGE_DETECTED)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_BAD_PIXEL)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
		imageLidar.at<cv::Vec3b>(y, x) = colorVector.at(0);
	}
	else if (value < 0)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else if (value > maxAmplitudeValue)
	{
		imageLidar.at<cv::Vec3b>(y, x)[0] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[1] = 0;
		imageLidar.at<cv::Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = value * indexAmplitudeFactorColor - 1;
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = 0;
		}
		else if( index > (int)colorVector.size() ){
			index = colorVector.size()-1;
		}
		
		imageLidar.at<cv::Vec3b>(y, x) = colorVector.at(index);
	}

}


void Nsl3130Driver::updateTemperature(double temperature)
{
#if 1
	std::ignore = temperature;
#else
    //ROS_DEBUG("Temperature: %2.2f", temperature);
    msgTemperature.header.frame_id = "sensor_frame";
    msgTemperature.variance = 0.0; //0.05 ?
    msgTemperature.header.stamp = ros::Time::now();
    msgTemperature.temperature = temperature;
    temperaturePublisher.publish(msgTemperature);
    msgTemperature.header.seq++;
#endif	
}


void Nsl3130Driver::updateGrayscaleFrame(std::shared_ptr<com_lib::Nsl3130Image> image)
{
	int offset = image->getOffset();
	auto data_stamp = _ros_clock.now();
	cv::Mat amplitudeLidar(image->getHeight(), image->getWidth(), CV_8UC3, cv::Scalar(255, 255, 255));

	frameAddCnt++;

    if(gSettings->enableTemperature){
        updateTemperature(image->getTemperature());
    }

    if(gSettings->enableImages)
    {
        //img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = data_stamp;
        img16_1.header.frame_id = strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;
        uint numPix = img16_1.width * img16_1.height;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(static_cast<unsigned long>(numPix*2));

			dist2BData.resize(numPix);
			ampl2BData.resize(numPix);
			gray2BData.resize(numPix);
        }

		dist2BData.clear();
		ampl2BData.clear();
		gray2BData.clear();

		int maxHeight = image->getHeight();
		int maxWidth = image->getWidth();
		std::vector<uint8_t>& data = image->getData();

	    for (int y = 0, index = 0; y < maxHeight; y ++)
		{
			for (int x = 0; x < maxWidth; x++, index++)
			{
				int pixelAmplitude = (data[2*index+1+offset] << 8) + data[2*index+0+offset];
	            img16_1.data[2*index+0] = data[2*index+0+offset];
	            img16_1.data[2*index+1] = data[2*index+1+offset];

				
				setGrayScaledColor(amplitudeLidar, x, y, pixelAmplitude, 2048);

				gray2BData[index] = pixelAmplitude;
			}
    	}

        imagePublisher1->publish(img16_1);

    } //end if enableImages

	if(settings.cvShow)
	{
		getMouseEvent(mouseXpos, mouseYpos);
		
		amplitudeLidar = addDistanceInfo(amplitudeLidar, image);
		
		cv::imshow(winName, amplitudeLidar);
		cv::waitKey(1);
	}

}


void Nsl3130Driver::updateDistanceFrame(std::shared_ptr<com_lib::Nsl3130Image> image)
{    
    std::vector<uint8_t>& data = image->getData();
	int offset = image->getOffset();
	auto data_stamp = _ros_clock.now();

	frameAddCnt++;

    cv::Mat imageLidar(image->getHeight(), image->getWidth(), CV_8UC3, cv::Scalar(255, 255, 255));

    if(gSettings->enableTemperature){
        updateTemperature(image->getTemperature());
    }

	printf("updateDistanceFrame\n");

    if(gSettings->enableImages)
    {
        //img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = data_stamp;
        img16_1.header.frame_id = strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;//pixelSize;
        img16_1.is_bigendian = 0;
        uint numPix = img16_1.width * img16_1.height;
        uint dataSize =  numPix * 2;//pixelSize;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(static_cast<ulong>(dataSize));

			dist2BData.resize(numPix);
			ampl2BData.resize(numPix);
			gray2BData.resize(numPix);
        }

		dist2BData.clear();
		ampl2BData.clear();
		gray2BData.clear();

		printf("updateDistanceFrame 2\n");

		int maxHeight = image->getHeight();
		int maxWidth = image->getWidth();

	    for (int y = 0, index = 0; y < maxHeight; y ++)
		{
			for (int x = 0; x < maxWidth; x++, index++)
			{
				int pixelDistance = (data[2*index+1+offset] << 8) + data[2*index+0+offset];
	            img16_1.data[2*index+0] = data[2*index+0+offset];
	            img16_1.data[2*index+1] = data[2*index+1+offset];

				
				setDistanceColor(imageLidar, x, y, pixelDistance);

				dist2BData[index] = pixelDistance;
			}
    	}

		printf("updateDistanceFrame3 \n");
        imagePublisher1->publish(img16_1);
    }

	printf("updateDistanceFrame 4\n");
	publisherPointCloud(image);


	if(settings.cvShow)
	{
		getMouseEvent(mouseXpos, mouseYpos);
		
		imageLidar = addDistanceInfo(imageLidar, image);

		cv::imshow(winName, imageLidar);
		cv::waitKey(1);
	}
	printf("updateDistanceFrame 5\n");

}


void Nsl3130Driver::updateDistanceAmplitudeFrame(std::shared_ptr<Nsl3130Image> image)
{
    std::vector<uint8_t>& data = image->getData();
	int offset = image->getOffset();
	auto data_stamp = _ros_clock.now();

	frameAddCnt++;

    cv::Mat imageLidar(image->getHeight(), image->getWidth(), CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat amplitudeLidar(image->getHeight(), image->getWidth(), CV_8UC3, cv::Scalar(255, 255, 255));

    if(gSettings->enableTemperature){
        updateTemperature(image->getTemperature());
    }

    if(gSettings->enableImages)
    {
//        img16_1.header.seq = frameSeq;
        img16_1.header.stamp = data_stamp;
        img16_1.header.frame_id = strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;

        img16_2.header.stamp = data_stamp;
        img16_2.header.frame_id = strFrameID;
        img16_2.height = static_cast<uint32_t>(image->getHeight());
        img16_2.width = static_cast<uint32_t>(image->getWidth());
        img16_2.encoding = sensor_msgs::image_encodings::MONO16;
        img16_2.step = img16_2.width * 2;
        img16_2.is_bigendian = 0;

        uint numPix = img16_1.width * img16_1.height;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            imageSize16_2 = numPix;
			
            img16_1.data.resize(static_cast<ulong>(numPix) * 2);
            img16_2.data.resize(static_cast<ulong>(numPix) * 2);

			dist2BData.resize(numPix);
			ampl2BData.resize(numPix);
			gray2BData.resize(numPix);
        }

		dist2BData.clear();
		ampl2BData.clear();
		gray2BData.clear();

		int maxHeight = image->getHeight();
		int maxWidth = image->getWidth();

	    for (int y = 0, index = 0; y < maxHeight; y ++)
		{
			for (int x = 0; x < maxWidth; x++, index++)
			{
				int pixelDistance = (data[4*index+1+offset] << 8) + data[4*index+0+offset];
	            img16_1.data[2*index+0] = data[4*index+0+offset];
	            img16_1.data[2*index+1] = data[4*index+1+offset];

				int pixelAmplitude = (data[4*index+3+offset] << 8) + data[4*index+2+offset];
	            img16_2.data[2*index+0] = data[4*index+2+offset];
	            img16_2.data[2*index+1] = data[4*index+3+offset];
				
				setDistanceColor(imageLidar, x, y, pixelDistance);
				setAmplitudeColor(amplitudeLidar, x, y, pixelAmplitude);

				dist2BData[index] = pixelDistance;
				ampl2BData[index] = pixelAmplitude;
			}
    	}

        imagePublisher1->publish(img16_1);
        imagePublisher2->publish(img16_2);

    }

	publisherPointCloud(image);

	if(settings.cvShow)
	{
		getMouseEvent(mouseXpos, mouseYpos);

		cv::hconcat(imageLidar, amplitudeLidar, imageLidar);
		imageLidar = addDistanceInfo(imageLidar, image);

		cv::imshow(winName, imageLidar);
		cv::waitKey(1);
	}

}

void Nsl3130Driver::publisherPointCloud(std::shared_ptr<Nsl3130Image> image)
{
	auto data_stamp = _ros_clock.now();

    if(gSettings->enablePointCloud)
    {
        const uint nPixel = image->getWidth() * image->getHeight();
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;
		cloud->points.resize(nPixel);
		
		for(unsigned int k=0, l=0, y=0; y < image->getHeight(); y++)
		{
			for(unsigned int x=0; x < image->getWidth(); x++, k++, l+=2)
			{
				pcl::PointXYZI &p = cloud->points[k];

				int distance = dist2BData[k];

				if (distance > 0 && distance < gSettings->maxDistance)
				{
					double px, pz, py;
                    cartesian.transformPixel(x, y, (double)(distance), px, py, pz);
					p.x = static_cast<float>(pz / 1000.0); //mm -> m
					p.y = static_cast<float>(px / 1000.0);
					p.z = static_cast<float>(py / 1000.0);

					if(image->getDataType() == Nsl3130Image::DataType::DATA_DISTANCE_AMPLITUDE
						|| image->getDataType() == Nsl3130Image::DataType::DATA_DISTANCE_AMPLITUDE_GRAYSCALE ) p.intensity = static_cast<float>(ampl2BData[k]);
					else p.intensity = static_cast<float>(pz / 1000.0);
				}
				else
				{
					p.x = std::numeric_limits<float>::quiet_NaN();
					p.y = std::numeric_limits<float>::quiet_NaN();
					p.z = std::numeric_limits<float>::quiet_NaN();
					p.intensity = std::numeric_limits<float>::quiet_NaN();
				}		  

			}
		}
		
		sensor_msgs::msg::PointCloud2 msg;
		pcl::toROSMsg(*cloud, msg);
		msg.header.stamp = data_stamp;
		msg.header.frame_id = strFrameID;
		pointCloud2Publisher->publish(msg);  

	} //end if enablePointCloud

}

cv::Mat Nsl3130Driver::addDistanceInfo(cv::Mat distMat, std::shared_ptr<Nsl3130Image> image)
{
	unsigned int xpos = (unsigned int)mouseXpos;
	unsigned int ypos = (unsigned int)mouseYpos;

//	printf("mouseXpos = %d mouseYpos = %d width = %d height = %d type = %d\n", mouseXpos, mouseYpos, image->getWidth(), image->getHeight(), image->getDataType());
	
	if( (ypos > 0 && ypos < image->getHeight())){
		// mouseXpos, mouseYpos
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, cv::Scalar(255, 255, 255));

		cv::line(distMat, cv::Point(xpos-10, ypos), cv::Point(xpos+10, ypos), cv::Scalar(255, 255, 0), 2);
		cv::line(distMat, cv::Point(xpos, ypos-10), cv::Point(xpos, ypos+10), cv::Scalar(255, 255, 0), 2);

		if( xpos >= image->getWidth()*2 ){
			xpos -= image->getWidth()*2;
		}
		else if( xpos >= image->getWidth() ){
			xpos -= image->getWidth();
		}

		std::string dist_caption;

		int real_xpos = xpos;
		int real_dist = dist2BData[ypos*image->getWidth() + real_xpos];
		if( real_dist > NSL3130_LIMIT_FOR_VALID_DATA ){

			if( real_dist == NSL3130_ADC_OVERFLOW )
				dist_caption = cv::format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( real_dist == NSL3130_SATURATION )
				dist_caption = cv::format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( real_dist == NSL3130_BAD_PIXEL )
				dist_caption = cv::format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( real_dist == NSL3130_INTERFERENCE )
				dist_caption = cv::format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( real_dist == NSL3130_EDGE_DETECTED )
				dist_caption = cv::format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist_caption = cv::format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			if( image->getDataType() == Nsl3130Image::DataType::DATA_DISTANCE_AMPLITUDE ) dist_caption = cv::format("X:%d,Y:%d %dmm/%dlsb", xpos, ypos, dist2BData[ypos*image->getWidth() + real_xpos], ampl2BData[ypos*image->getWidth() + real_xpos]);
			else if( image->getDataType() == Nsl3130Image::DataType::DATA_DISTANCE_GRAYSCALE ) dist_caption = cv::format("X:%d,Y:%d %dmm/%dlsb", xpos, ypos, dist2BData[ypos*image->getWidth() + real_xpos], gray2BData[ypos*image->getWidth() + real_xpos]);
			else if( image->getDataType() == Nsl3130Image::DataType::DATA_DISTANCE_AMPLITUDE_GRAYSCALE ) dist_caption = cv::format("X:%d,Y:%d %dmm/%dlsb/%dlsb", xpos, ypos, dist2BData[ypos*image->getWidth() + real_xpos], ampl2BData[ypos*image->getWidth() + real_xpos], gray2BData[ypos*image->getWidth() + real_xpos]);
			else if( image->getDataType() == Nsl3130Image::DataType::DATA_GRAYSCALE )	dist_caption = cv::format("X:%d,Y:%d %dlsb", xpos, ypos, gray2BData[ypos*image->getWidth() + real_xpos]);
			else	dist_caption = cv::format("X:%d,Y:%d %dmm", xpos, ypos, dist2BData[ypos*image->getWidth() + real_xpos]);
		}

		putText(infoImage, dist_caption.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));
		cv::vconcat(distMat, infoImage, distMat);
	}
	else{
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, cv::Scalar(255, 255, 255));

		// frameCnt
		std::string dist_caption;
		dist_caption = cv::format("frame rate : %d fps", frameCnt);
		putText(infoImage, dist_caption.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));

		cv::vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}


