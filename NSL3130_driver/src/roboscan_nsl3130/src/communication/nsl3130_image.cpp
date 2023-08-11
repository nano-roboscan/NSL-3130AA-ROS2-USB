#include "communication/nsl3130_image.h"
#include "util.h"
#include "rclcpp/rclcpp.hpp"

using namespace std;

namespace com_lib
{

Nsl3130Image::Nsl3130Image(const vector<uint8_t> &data)
{
    this->data = data;
    version = data[0];
    dataType = Util::getUint16BigEndian(data, 1); //answer or data
    width  = Util::getUint16BigEndian(data, 3);
    height = Util::getUint16BigEndian(data, 5);
    roiX0  = Util::getUint16BigEndian(data, 7);
    roiY0  = Util::getUint16BigEndian(data, 9);
    roiX1  = Util::getUint16BigEndian(data, 11);
    roiY1  = Util::getUint16BigEndian(data, 13);

    temperature = static_cast<double>(Util::getInt16BigEndian(data, 21)) / 100.0;
    offset = Util::getInt16BigEndian(data, 23); //offset where measurement data begins

 	
//	printf("Nsl3130Image header: v= %d type=%d w= %d h= %d temp= %2.2f offset= %d roiX0 = %d, roiY0= %d, roiX1= %d, roiY1 = %d\n", 
  //  			version, dataType, width, height, temperature, offset, roiX0, roiY0, roiX1, roiY1);
}

unsigned int Nsl3130Image::getWidth() const
{
    return width;
}

unsigned int Nsl3130Image::getHeight() const
{
    return height;
}

unsigned int Nsl3130Image::getLeftX() const
{
    return roiX0;
}

unsigned int Nsl3130Image::getTopY() const
{
    return roiY0;
}

double Nsl3130Image::getTemperature() const
{
    return temperature;
}

int Nsl3130Image::getOffset() const
{
	return offset;
}

int Nsl3130Image::getDataType() const
{
	return dataType;
}

std::vector<uint8_t>& Nsl3130Image::getData()
{
    return data;
}

uint8_t* Nsl3130Image::getData(unsigned int index)
{
    return  (uint8_t *)(&data.data()[offset + index]);
}

} //end namespace com_lib
