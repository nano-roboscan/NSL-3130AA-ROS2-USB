#ifndef __NSL3130_IMAGE_H__
#define __NSL3130_IMAGE_H__

#include "communication_constants.h"
#include <vector>
#include <string>


namespace com_lib
{

class Nsl3130Image
{
public:
    enum ImageType_e { GRAYSCALE, DISTANCE, DISTANCE_AMPLITUDE };
	enum DataType { DATA_DISTANCE_AMPLITUDE, DATA_DISTANCE, DATA_AMPLITUDE, DATA_GRAYSCALE, DATA_DCS, DATA_DISTANCE_GRAYSCALE, DATA_DISTANCE_AMPLITUDE_GRAYSCALE };

    Nsl3130Image(const std::vector<uint8_t> &data);    

    unsigned int getWidth() const;
    unsigned int getHeight() const;
    unsigned int getLeftX() const;
    unsigned int getTopY() const;
    double getTemperature() const;
	int getOffset() const;
	int getDataType() const;
    std::vector<uint8_t>& getData();
    uint8_t* getData(unsigned int index);

protected:
    std::vector<uint8_t> data;

private:    
    uint8_t version;
    uint16_t dataType;
    uint16_t width;
    uint16_t height;
    uint16_t roiX0;
    uint16_t roiY0;
    uint16_t roiX1;
    uint16_t roiY1;
    uint16_t offset;
    double temperature;
};

}

#endif // __NSL3130_IMAGE_H__
