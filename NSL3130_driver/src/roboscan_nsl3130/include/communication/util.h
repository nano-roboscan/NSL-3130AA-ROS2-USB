/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup util Utility
 * @brief Utility class containing static helper functions
 * @ingroup communication
 *
 * @{
 */
#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <vector>

namespace com_lib
{
  //! Utility class
  /*!
   * This class offers some helper functions needed in different other classes. These helper
   * functions are static, so no instance is needed.
   */
  class Util
  {
    public:
      Util();
      static int16_t  getInt16BigEndian(uint8_t *array, const unsigned int index);
      static uint32_t getUint16BigEndian(uint8_t *array, const unsigned int index);
      static uint32_t getUint32BigEndian(uint8_t *array, const unsigned int index);
      static int16_t  getInt16BigEndian(const std::vector<uint8_t> &array, const unsigned int index);
      static uint32_t getUint16BigEndian(const std::vector<uint8_t> &array, const unsigned int index);
      static uint32_t getUint32BigEndian(const std::vector<uint8_t> &array, const unsigned int index);
      static void setUint16BigEndian(uint8_t *buffer, const unsigned int index, const unsigned int value);
      static void setInt16BigEndian(uint8_t *buffer, const unsigned int index, const int value);
      static void setUint24BigEndian(uint8_t *buffer, const unsigned int index, const unsigned int value);
      static void setUint32BigEndian(uint8_t *buffer, const unsigned int index, const unsigned int value);
  };

}

#endif // UTIL_H

/** @} */
