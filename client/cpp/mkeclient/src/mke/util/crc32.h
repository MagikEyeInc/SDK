#ifndef _CRC32_H_
#define _CRC32_H_

#include <stdint.h>
#include <stdlib.h> 

namespace mke {
namespace util {
  
/**
* @brief Crc32 provides implementation of CRC32 checksum algorithm
*
*/
class Crc32 {
private:
static uint32_t crc_32_tab[];

static inline uint32_t updc32(const uint8_t octet, const uint32_t crc) {
  return (crc_32_tab[((crc) ^ ((uint8_t) octet)) & 0xFF] ^ ((crc) >> 8));
}

public:
  static inline uint32_t crcBuffer(const void *buffer, const size_t len, const uint32_t seed = 0xFFFFFFFF) {
      uint32_t oldcrc32 = seed;
      uint8_t * buf = (uint8_t *) buffer;
      
      for (int i = 0; i < len; i++)
      {
            oldcrc32 = updc32(*buf, oldcrc32);
            buf++;
      }

      return oldcrc32;
  }
};
  
} /* namespace util */
} /* namespace mke */

#endif /* _CRC32_H_ */
