
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

//#define USE_CRC_FLASH   // CRC tables in flash rather than EAM
#define USE_CRC_TABLES  // fast CRC .. if you have the flash or RAM to spare for the CRC tables

#ifndef USE_CRC_FLASH
	void FASTCALL make_CRC16_table(void);
#endif
uint16_t FASTCALL CRC16(const uint16_t crc, const uint8_t data);
uint16_t FASTCALL CRC16_block(uint16_t crc, const void *data, int n);

#ifdef __cplusplus
	}
#endif

#endif
