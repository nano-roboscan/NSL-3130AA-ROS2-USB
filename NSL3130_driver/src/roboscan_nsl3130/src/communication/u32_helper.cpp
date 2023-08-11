#include "u32_helper.h"
#include <iostream>
#include "rclcpp/rclcpp.hpp"

namespace com_lib
{

U32Helper::U32Helper()
{
  value = 0;
}

uint32_t U32Helper::getValue()
{
  return value;
}

void U32Helper::onReceivedData(const uint32_t value)
{  
	printf("U32Helper:onReceivedData : %x\n", value);
	this->value = value;
}

uint16_t U32Helper::getValueLsb()
{	
	uint32_t Lsb = (value & 0xffff);
	unsigned int byte0 = (Lsb & 0xffff) >> 8;
	unsigned int byte1 = (Lsb & 0xffff) & 0xFF;
	
	uint32_t retValue = static_cast<uint32_t>((byte0 << 8) | byte1);
	return retValue;
}

uint16_t U32Helper::getValueMsb()
{
	uint32_t Msb = ((value >> 16) & 0xffff);
	unsigned int byte0 = (Msb & 0xffff) >> 8;
	unsigned int byte1 = (Msb & 0xffff) & 0xFF;
	
	uint32_t retValue = static_cast<uint32_t>((byte0 << 8) | byte1);
	return retValue;
}

}
