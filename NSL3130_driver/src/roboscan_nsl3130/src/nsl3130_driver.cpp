
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

static Settings settings_callback;
Settings Nsl3130Driver::settings;


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


	settings.imageType = Nsl3130Image::ImageType_e::DISTANCE_AMPLITUDE;
	settings.lenstype = 1;
	settings.frameRate = 20; //fps
	settings.lensCenterOffsetX = 0;
	settings.lensCenterOffsetY = 0;
	settings.hdrMode = 0;
	settings.integrationTimeTOF1 = 1500;
	settings.integrationTimeTOF2 = 500;
	settings.integrationTimeTOF3 = 50;
	settings.integrationTimeGray = 100;
	settings.modFrequency = 1;	// 12Mhz
	settings.modChannel = 0;

	settings.medianFilter = false;
	settings.averageFilter = false;
	settings.kalmanFactor = 0;
	settings.kalmanThreshold = 300;
	settings.edgeThreshold = 0;
	settings.interferenceDetectionLimit = 0;
	settings.interferenceDetectionUseLastValue = 0;

	settings.minAmplitude = 50;
	settings.minDistance = 30;
	settings.maxDistance = 12500;


	settings.enableCartesian   = true;
	settings.enableTemperature = false;
	settings.enableImages      = true;
	settings.enablePointCloud  = true;

	settings.roi_leftX   = 0;
	settings.roi_rightX  = 319;
	settings.roi_topY = 0;
	settings.roi_bottomY = 239;

	settings.startStream = true;	// auto start
	settings.runVideo = false;
	settings.triggerSingleShot = false;
	settings.updateParam = true;
	settings.cvShow = false;
	settings.changedCvShow = false;

	parameterInit();

	auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
	imagePublisher1 = create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile); 
	imagePublisher2 = create_publisher<sensor_msgs::msg::Image>("roboscanAmplitude", qos_profile); 
	pointCloud2Publisher = create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile); 

	initCommunication();

	communication.sigReceivedGrayscale.connect(boost::bind(&Nsl3130Driver::updateGrayscaleFrame, this, _1));
	communication.sigReceivedDistance.connect(boost::bind(&Nsl3130Driver::updateDistanceFrame, this, _1));
	communication.sigReceivedDistanceAmplitude.connect(boost::bind(&Nsl3130Driver::updateDistanceAmplitudeFrame, this, _1));    

	settings.updateParam = true;
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
	rcl_interfaces::msg::ParameterDescriptor descriptor;
	rcl_interfaces::msg::IntegerRange range;
	rcl_interfaces::msg::ParameterDescriptor descriptorFloat;
	rcl_interfaces::msg::FloatingPointRange floating_range;


	this->declare_parameter<bool>("A. startStream", settings.startStream);	
	this->declare_parameter<bool>("B. triggerSingleShot", settings.triggerSingleShot);
	this->declare_parameter<bool>("C. cvShow", settings.cvShow);


	range.set__from_value(0).set__to_value(2).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("D. imageType", settings.imageType, descriptor);

	range.set__from_value(0).set__to_value(3).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("E. modFrequency", settings.modFrequency, descriptor);

	range.set__from_value(0).set__to_value(2).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("F. Hdr", 0, descriptor);	

	range.set__from_value(0).set__to_value(4000).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("G. integrationTime0", settings.integrationTimeTOF1, descriptor);

	range.set__from_value(0).set__to_value(4000).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("H. integrationTime1", settings.integrationTimeTOF2, descriptor);

	range.set__from_value(0).set__to_value(4000).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("I. integrationTime2", settings.integrationTimeTOF3, descriptor);

	range.set__from_value(0).set__to_value(40000).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("J. integrationTimeGray", settings.integrationTimeGray, descriptor);

	floating_range.set__from_value(0.0).set__to_value(1.0).set__step(0.1);
	descriptorFloat.floating_point_range= {floating_range};
	this->declare_parameter("K. temporalFilterFactor", settings.kalmanFactor, descriptorFloat);

	range.set__from_value(0).set__to_value(1000).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("L. temporalFilterThreshold", settings.kalmanThreshold, descriptor);

	this->declare_parameter<bool>("M. medianFilter", settings.medianFilter);
	this->declare_parameter<bool>("N. averageFilter", settings.averageFilter);

	range.set__from_value(0).set__to_value(5000).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("O. edgeThreshold", settings.edgeThreshold, descriptor);

	range.set__from_value(0).set__to_value(1000).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("P. minAmplitude", settings.minAmplitude, descriptor);

	range.set__from_value(0).set__to_value(1000).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("Q. minDistance", settings.minDistance, descriptor);

	range.set__from_value(0).set__to_value(50000).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("R. maxDistance", settings.maxDistance, descriptor);

	range.set__from_value(0).set__to_value(124).set__step(4);
	descriptor.integer_range= {range};
	this->declare_parameter("S. roiLeftX", settings.roi_leftX, descriptor);

	range.set__from_value(0).set__to_value(116).set__step(2);
	descriptor.integer_range= {range};
	this->declare_parameter("T. roiTopY", settings.roi_topY, descriptor);

	range.set__from_value(131).set__to_value(319).set__step(4);
	descriptor.integer_range= {range};
	this->declare_parameter<int>("U. roiRightX", settings.roi_rightX, descriptor);

//	range.set__from_value(123).set__to_value(239).set__step(2);
//	descriptor.integer_range= {range};
//	this->declare_parameter("V. roiBottomY", settings.roi_bottomY, descriptor);

	range.set__from_value(0).set__to_value(2).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("W. lensType", settings.lenstype, descriptor);

	range.set__from_value(0).set__to_value(20).set__step(1);
	descriptor.integer_range= {range};
	this->declare_parameter("X. frameRate", settings.frameRate, descriptor);


	memcpy(&settings_callback, &settings, sizeof(settings_callback));
}


rcl_interfaces::msg::SetParametersResult Nsl3130Driver::parametersCallback( const std::vector<rclcpp::Parameter> &parameters)
{
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";
	// Here update class attributes, do some actions, etc.
	bool chanedRoiTopY = false;
	
	for (const auto &param: parameters)
	{
		if (param.get_name() == "A. startStream")
		{
			settings_callback.startStream = param.as_bool();
		}
		else if (param.get_name() == "B. triggerSingleShot")
		{
			settings_callback.triggerSingleShot = param.as_bool();
		}
		else if (param.get_name() == "C. cvShow")
		{
			bool showCv = param.as_bool();
			if( settings_callback.cvShow != showCv ){
				settings_callback.cvShow = showCv;
				settings_callback.changedCvShow = true;
			}
		}
		else if (param.get_name() == "D. imageType")
		{
			int imgType = param.as_int();
			
			if( settings_callback.imageType != imgType ){
				settings_callback.imageType = imgType;
				settings_callback.changedCvShow = true;
			}
		}
		else if (param.get_name() == "E. modFrequency")
		{
			settings_callback.modFrequency = param.as_int();
		}
		else if (param.get_name() == "F. Hdr")
		{
			settings_callback.hdrMode = param.as_int();
		}
		else if (param.get_name() == "G. integrationTime0")
		{
			settings_callback.integrationTimeTOF1 = param.as_int();
		}
		else if (param.get_name() == "H. integrationTime1")
		{
			settings_callback.integrationTimeTOF2 = param.as_int();
		}
		else if (param.get_name() == "I. integrationTime2")
		{
			settings_callback.integrationTimeTOF3 = param.as_int();
		}
		else if (param.get_name() == "J. integrationTimeGray")
		{
			settings_callback.integrationTimeGray = param.as_int();
		}
		else if (param.get_name() == "K. temporalFilterFactor")
		{
			settings_callback.kalmanFactor = param.as_double();
		}
		else if (param.get_name() == "L. temporalFilterThreshold")
		{
			settings_callback.kalmanThreshold = param.as_int();
		}
		else if (param.get_name() == "M. medianFilter")
		{
			settings_callback.medianFilter = param.as_bool();
		}
		else if (param.get_name() == "N. averageFilter")
		{
			settings_callback.averageFilter = param.as_bool();
		}
		else if (param.get_name() == "O. edgeThreshold")
		{
			settings_callback.edgeThreshold = param.as_int();
		}
		else if (param.get_name() == "P. minAmplitude")
		{
			settings_callback.minAmplitude = param.as_int();
		}
		else if (param.get_name() == "Q. minDistance")
		{
			settings_callback.minDistance = param.as_int();
		}
		else if (param.get_name() == "R. maxDistance")
		{
			settings_callback.maxDistance = param.as_int();
			if( settings_callback.maxDistance == 0 ) settings_callback.maxDistance = 1;
		}
		else if (param.get_name() == "S. roiLeftX")
		{
			settings_callback.roi_leftX= param.as_int();
		}
		else if (param.get_name() == "T. roiTopY")
		{
			int roi_topY = param.as_int();
			if( chanedRoiTopY == false && settings_callback.roi_topY != roi_topY ){
				int step = 239-roi_topY;
				settings_callback.roi_topY = roi_topY;
				settings_callback.roi_bottomY = step;
				chanedRoiTopY = true;
			}
		}
		else if (param.get_name() == "U. roiRightX")
		{
			settings_callback.roi_rightX= param.as_int();
		}
		else if (param.get_name() == "V. roiBottomY")
		{
			int roi_bottomY= param.as_int();
			if( chanedRoiTopY == false && settings_callback.roi_bottomY != roi_bottomY )
			{
				int step = 239-roi_bottomY;
				settings_callback.roi_bottomY = roi_bottomY;
				settings_callback.roi_topY = step;
				chanedRoiTopY = true;
			}
		}
		else if (param.get_name() == "W. lensType")
		{
			settings_callback.lenstype= param.as_int();
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

	if(settings.runVideo && !settings.updateParam){
		updateData(); //streaming

	}else if(settings.updateParam){
		setParameters(); //update parameters

		if(settings.triggerSingleShot && settings.triggerSingleShot != lastSingleShot)
			updateData(); //trigger single shot

		lastSingleShot = settings.triggerSingleShot;
	}
}

void Nsl3130Driver::setParameters()
{
	if(settings.updateParam)
	{
		memcpy(&settings, &settings_callback, sizeof(settings_callback));
		settings.updateParam = false;

		printf("update parameters hdr = %d\n", settings.hdrMode);

		framePeriod = 1.0 / settings.frameRate * 1000.0f; // msec

		int modFrequency = settings.modFrequency == 0 ? 1 : settings.modFrequency == 1 ? 0 : settings.modFrequency > 3 ? 3 : settings.modFrequency;
		int modChannel = settings.modChannel < 0 ? 0 : settings.modChannel > 15 ? 15 : settings.modChannel;

		communication.setHDRMode(settings.hdrMode);
		communication.setIntegrationTime3d(settings.integrationTimeTOF1, settings.integrationTimeTOF2, settings.integrationTimeTOF3, settings.integrationTimeGray);        
		communication.setMinimalAmplitude(settings.minAmplitude);
		communication.setModulationFrequency(modFrequency, modChannel);
		communication.setFilter(settings.medianFilter, settings.averageFilter, 1000.0 * settings.kalmanFactor, settings.kalmanThreshold,
								settings.edgeThreshold, settings.interferenceDetectionLimit, settings.interferenceDetectionUseLastValue);
		communication.setRoi(settings.roi_leftX, settings.roi_topY, settings.roi_rightX, settings.roi_bottomY);

		cartesian.initLensTransform(sensorPointSizeMM, numCols, numRows, settings.lensCenterOffsetX, settings.lensCenterOffsetY, settings.lenstype);
		oldLensCenterOffsetX = settings.lensCenterOffsetX;
		oldLensCenterOffsetY = settings.lensCenterOffsetY;	

		if(settings.startStream)
			settings.runVideo = true;
		else
			settings.runVideo = false;

		setWinName();
	}
}


void Nsl3130Driver::updateData()
{
	std::chrono::system_clock::time_point timeNow = std::chrono::system_clock::now();
	std::chrono::milliseconds elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - timeLast);

	if(elapsed_time.count() >= framePeriod )
	{
		timeLast = timeNow;

		switch(settings.imageType)
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

	}
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

double Nsl3130Driver::interpolate( double x, double x0, double y0, double x1, double y1)
{
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
	else if (value < settings.minDistance)
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
		int index = colorVector.size() - (value*(NSL3130_NUM_COLORS / (double)settings.maxDistance));
		if( index < 0 || index == NSL3130_NUM_COLORS ){
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

	if(settings.enableTemperature){
		updateTemperature(image->getTemperature());
	}

	if(settings.enableImages)
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

	if(settings.enableTemperature){
		updateTemperature(image->getTemperature());
	}

	if(settings.enableImages)
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

		imagePublisher1->publish(img16_1);
	}

	publisherPointCloud(image);

	if(settings.cvShow)
	{
		getMouseEvent(mouseXpos, mouseYpos);
		imageLidar = addDistanceInfo(imageLidar, image);

		cv::imshow(winName, imageLidar);
		cv::waitKey(1);
	}
}


void Nsl3130Driver::updateDistanceAmplitudeFrame(std::shared_ptr<Nsl3130Image> image)
{
	std::vector<uint8_t>& data = image->getData();
	int offset = image->getOffset();
	auto data_stamp = _ros_clock.now();

	frameAddCnt++;

	cv::Mat imageLidar(image->getHeight(), image->getWidth(), CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat amplitudeLidar(image->getHeight(), image->getWidth(), CV_8UC3, cv::Scalar(255, 255, 255));

	if(settings.enableTemperature){
		updateTemperature(image->getTemperature());
	}

	if(settings.enableImages)
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

	if(settings.enablePointCloud)
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

				if (distance > settings.minDistance && distance < settings.maxDistance)
				{
					double px, pz, py;
					cartesian.transformPixel(x, y, (double)(distance), px, py, pz);
					p.x = static_cast<float>(pz / 1000.0); //mm -> m
					p.y = static_cast<float>(px / 1000.0);
					p.z = static_cast<float>(py / 1000.0);

					if(image->getDataType() == Nsl3130Image::DataType::DATA_DISTANCE_AMPLITUDE || image->getDataType() == Nsl3130Image::DataType::DATA_DISTANCE_AMPLITUDE_GRAYSCALE ) 
						p.intensity = static_cast<float>(ampl2BData[k]);
					else 
						p.intensity = static_cast<float>(pz / 1000.0);
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
	}

}

cv::Mat Nsl3130Driver::addDistanceInfo(cv::Mat distMat, std::shared_ptr<Nsl3130Image> image)
{
	unsigned int xpos = (unsigned int)mouseXpos;
	unsigned int ypos = (unsigned int)mouseYpos;

//	printf("mouseXpos = %d mouseYpos = %d width = %d height = %d type = %d\n", mouseXpos, mouseYpos, image->getWidth(), image->getHeight(), image->getDataType());
	
	if( (ypos > 0 && ypos < image->getHeight()))
	{
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


