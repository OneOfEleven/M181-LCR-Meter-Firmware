
// 1o11

#ifndef crcH
#define crcH

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

#if (__BORLANDC__)
	#define FASTCALL __fastcall
#else
	#define FASTCALL
#endif

//#define USE_CRC_FLASH   // CRC tables in flash rather than RAM, RAM access is faster though
#define USE_CRC_TABLES    // CRC tables (fast)
//#define CRC_TABLE_4       // use smaller tables

#define CRC_FWD_16         // this
//#define CRC_REV_16       // or this

//#define CRC_FWD_32       // this
//#define CRC_REV_32       // or this

#if defined(CRC_FWD_16) || defined(CRC_REV_16)
	#ifndef USE_CRC_FLASH
		void FASTCALL make_CRC16_table(void);
	#endif
	uint16_t FASTCALL CRC16(uint16_t crc, const uint8_t data);
	uint16_t FASTCALL CRC16_block(uint16_t crc, const void *data, unsigned int n);
#endif

#if defined(CRC_FWD_32) || defined(CRC_REV_32)
	#ifndef USE_CRC_FLASH
		void FASTCALL make_CRC32_table(void);
	#endif
	uint32_t FASTCALL CRC32(uint32_t crc, const uint8_t data);
	uint32_t FASTCALL CRC32_block(uint32_t crc, const void *data, unsigned int n);
#endif

#ifdef __cplusplus
	}
#endif

#endif
