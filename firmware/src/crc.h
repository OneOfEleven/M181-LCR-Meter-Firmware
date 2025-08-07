
// 1o11

#ifndef crcH
#define crcH

#ifdef __cplusplus
	extern "C" {
#endif

#if (__BORLANDC__)
	#define FASTCALL __fastcall
#else
	#define FASTCALL
#endif

#include <stdint.h>

//#define USE_CRC_FLASH   // CRC tables in flash rather than RAM, RAM access is faster though
#define USE_CRC_TABLES    // CRC tables (fast)
//#define CRC_TABLE_4       // use smaller tables

#ifndef USE_CRC_FLASH
	void FASTCALL make_CRC16_table(void);
#endif
uint16_t FASTCALL CRC16(const uint16_t crc, const uint8_t data);
uint16_t FASTCALL CRC16_block(uint16_t crc, const void *data, unsigned int n);

//uint32_t FASTCALL crc32(uint32_t *data, uint32_t count, const uint8_t reset);

#ifdef __cplusplus
	}
#endif

#endif
