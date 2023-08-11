#include "util.h"

#include <iostream>

using namespace std;

namespace com_lib
{

  Util::Util()
  {

  }

  uint32_t Util::getUint32BigEndian(uint8_t *array, const unsigned int index)
  {
      unsigned int byte0 = array[index];
      unsigned int byte1 = array[index+1];
      unsigned int byte2 = array[index+2];
      unsigned int byte3 = array[index+3];
      uint32_t value = (byte0 << 24) | (byte1 << 16) | (byte2 << 8) | byte3;
      return value;
  }

  uint32_t Util::getUint16BigEndian(uint8_t *array, const unsigned int index)
  {
      unsigned int byte0 = array[index];
      unsigned int byte1 = array[index+1];

      int32_t value = static_cast<int32_t>((byte0 << 8) | byte1);
      return value;
  }

  int16_t Util::getInt16BigEndian(uint8_t *array, const unsigned int index)
  {
      unsigned int byte0 = array[index];
      unsigned int byte1 = array[index+1];

      int16_t value = static_cast<int16_t>((byte0 << 8) | byte1);
      return value;
  }


  uint32_t Util::getUint32BigEndian(const std::vector<uint8_t> &array, const unsigned int index)
  {
      unsigned int byte0 = array.at(index);
      unsigned int byte1 = array.at(index+1);
      unsigned int byte2 = array.at(index+2);
      unsigned int byte3 = array.at(index+3);
      uint32_t value = (byte0 << 24) | (byte1 << 16) | (byte2 << 8) | byte3;
      return value;
  }

  uint32_t Util::getUint16BigEndian(const std::vector<uint8_t> &array, const unsigned int index)
  {
      unsigned int byte0 = array[index];
      unsigned int byte1 = array[index+1];

      int32_t value = static_cast<int32_t>((byte0 << 8) | byte1);
      return value;
  }

  int16_t Util::getInt16BigEndian(const std::vector<uint8_t> &array, const unsigned int index)
  {
      unsigned int byte0 = array[index];
      unsigned int byte1 = array[index+1];

      int16_t value = static_cast<int16_t>((byte0 << 8) | byte1);
      return value;
  }


  void Util::setUint16BigEndian(uint8_t *buffer, const unsigned int index, const unsigned int value)
  {
    buffer[index] = (value >> 8) & 0xFF;
    buffer[index+1] = (value ) & 0xFF;
  }

  void Util::setInt16BigEndian(uint8_t *buffer, const unsigned int index, const int value)
  {
    buffer[index] = (value >> 8) & 0xFF;
    buffer[index+1] = (value) & 0xFF;
  }

  void Util::setUint24BigEndian(uint8_t *buffer, const unsigned int index, const unsigned int value)
  {
    buffer[index] = (value >> 16) & 0xFF;
    buffer[index+1] = (value >> 8) & 0xFF;
    buffer[index+2] = (value) & 0xFF;
  }

  void Util::setUint32BigEndian(uint8_t *buffer, const unsigned int index, const unsigned int value)
  {
    buffer[index] =   (value >> 24) & 0xFF;
    buffer[index+1] = (value >> 16) & 0xFF;
    buffer[index+2] = (value >> 8) & 0xFF;
    buffer[index+3] = (value >> 0) & 0xFF;
  }

}
