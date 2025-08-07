
#include "crc.h"

#define CRC_TABLE_4

// x^16 + x^12 + x^5 + 1
//#define CRC_POLY16       0x1021
#define CRC_POLY16_REV   0x8408

#ifdef USE_CRC_TABLES

	#ifdef USE_CRC_FLASH

		#ifdef CRC_TABLE_4

			static const uint16_t CRC16_TABLE[] =
			{	// 0x8408
				0x0000, 0x8408, 0x8C18, 0x0810, 0x9C38, 0x1830, 0x1020, 0x9428,
				0xBC78, 0x3870, 0x3060, 0xB468, 0x2040, 0xA448, 0xAC58, 0x2850
			};

		#else

			static const uint16_t CRC16_TABLE[] =
			{	// 0x8408
				0x0000, 0x8408, 0x8C18, 0x0810, 0x9C38, 0x1830, 0x1020, 0x9428,
				0xBC78, 0x3870, 0x3060, 0xB468, 0x2040, 0xA448, 0xAC58, 0x2850,

				0xFCF8, 0x78F0, 0x70E0, 0xF4E8, 0x60C0, 0xE4C8, 0xECD8, 0x68D0,
				0x4080, 0xC488, 0xCC98, 0x4890, 0xDCB8, 0x58B0, 0x50A0, 0xD4A8,
				0x7DF8, 0xF9F0, 0xF1E0, 0x75E8, 0xE1C0, 0x65C8, 0x6DD8, 0xE9D0,
				0xC180, 0x4588, 0x4D98, 0xC990, 0x5DB8, 0xD9B0, 0xD1A0, 0x55A8,
				0x8100, 0x0508, 0x0D18, 0x8910, 0x1D38, 0x9930, 0x9120, 0x1528,
				0x3D78, 0xB970, 0xB160, 0x3568, 0xA140, 0x2548, 0x2D58, 0xA950,
				0xFBF0, 0x7FF8, 0x77E8, 0xF3E0, 0x67C8, 0xE3C0, 0xEBD0, 0x6FD8,
				0x4788, 0xC380, 0xCB90, 0x4F98, 0xDBB0, 0x5FB8, 0x57A8, 0xD3A0,
				0x0708, 0x8300, 0x8B10, 0x0F18, 0x9B30, 0x1F38, 0x1728, 0x9320,
				0xBB70, 0x3F78, 0x3768, 0xB360, 0x2748, 0xA340, 0xAB50, 0x2F58,
				0x8608, 0x0200, 0x0A10, 0x8E18, 0x1A30, 0x9E38, 0x9628, 0x1220,
				0x3A70, 0xBE78, 0xB668, 0x3260, 0xA648, 0x2240, 0x2A50, 0xAE58,
				0x7AF0, 0xFEF8, 0xF6E8, 0x72E0, 0xE6C8, 0x62C0, 0x6AD0, 0xEED8,
				0xC688, 0x4280, 0x4A90, 0xCE98, 0x5AB0, 0xDEB8, 0xD6A8, 0x52A0,
				0x73E8, 0xF7E0, 0xFFF0, 0x7BF8, 0xEFD0, 0x6BD8, 0x63C8, 0xE7C0,
				0xCF90, 0x4B98, 0x4388, 0xC780, 0x53A8, 0xD7A0, 0xDFB0, 0x5BB8,
				0x8F10, 0x0B18, 0x0308, 0x8700, 0x1328, 0x9720, 0x9F30, 0x1B38,
				0x3368, 0xB760, 0xBF70, 0x3B78, 0xAF50, 0x2B58, 0x2348, 0xA740,
				0x0E10, 0x8A18, 0x8208, 0x0600, 0x9228, 0x1620, 0x1E30, 0x9A38,
				0xB268, 0x3660, 0x3E70, 0xBA78, 0x2E50, 0xAA58, 0xA248, 0x2640,
				0xF2E8, 0x76E0, 0x7EF0, 0xFAF8, 0x6ED0, 0xEAD8, 0xE2C8, 0x66C0,
				0x4E90, 0xCA98, 0xC288, 0x4680, 0xD2A8, 0x56A0, 0x5EB0, 0xDAB8,
				0x8818, 0x0C10, 0x0400, 0x8008, 0x1420, 0x9028, 0x9838, 0x1C30,
				0x3460, 0xB068, 0xB878, 0x3C70, 0xA858, 0x2C50, 0x2440, 0xA048,
				0x74E0, 0xF0E8, 0xF8F8, 0x7CF0, 0xE8D8, 0x6CD0, 0x64C0, 0xE0C8,
				0xC898, 0x4C90, 0x4480, 0xC088, 0x54A0, 0xD0A8, 0xD8B8, 0x5CB0,
				0xF5E0, 0x71E8, 0x79F8, 0xFDF0, 0x69D8, 0xEDD0, 0xE5C0, 0x61C8,
				0x4998, 0xCD90, 0xC580, 0x4188, 0xD5A0, 0x51A8, 0x59B8, 0xDDB0,
				0x0918, 0x8D10, 0x8500, 0x0108, 0x9520, 0x1128, 0x1938, 0x9D30,
				0xB560, 0x3168, 0x3978, 0xBD70, 0x2958, 0xAD50, 0xA540, 0x2148
			};

		#endif

	#else

		uint8_t crc16_table_created = 0;

		#ifdef CRC_TABLE_4
			uint16_t CRC16_TABLE[16];
		#else
			uint16_t CRC16_TABLE[256];
		#endif

		void FASTCALL make_CRC16_table(void)
		{
			uint16_t i;

			if (crc16_table_created)
				return;

			#ifdef CRC_TABLE_4
				for (i = 0; i < 16; i++)
				{
					unsigned int k;
					uint16_t crc = i << 8;
					for (k = 8; k > 0; k--)
						crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY16_REV : crc << 1;
					CRC16_TABLE[i] = crc;
				}
			#else
				for (i = 0; i < 256; i++)
				{
					unsigned int k;
					uint16_t crc = i << 8;
					for (k = 8; k > 0; k--)
						crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY16_REV : crc << 1;
					CRC16_TABLE[i] = crc;
				}
			#endif

			crc16_table_created = 1;
		}

	#endif

	#ifdef CRC_TABLE_4

		uint16_t FASTCALL CRC16(uint16_t crc, const uint8_t data)
		{
			#ifndef USE_CRC_FLASH
				if (!crc16_table_created)
					make_CRC16_table();
			#endif

			crc ^= (uint16_t)data << 8;
			crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
			crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
			return crc;
		}

	#else

		uint16_t FASTCALL CRC16(const uint16_t crc, const uint8_t data)
		{
			#ifndef USE_CRC_FLASH
				if (!crc16_table_created)
					make_CRC16_table();
			#endif

			return (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ data];
		}

	#endif

	uint16_t FASTCALL CRC16_block(uint16_t crc, const void *data, unsigned int n)
	{
		const uint8_t *data8 = (uint8_t *)data;

		#ifndef USE_CRC_FLASH
			if (!crc16_table_created)
				make_CRC16_table();
		#endif

		while (n >= 4)
		{
			#ifdef CRC_TABLE_4
				crc ^= (uint16_t)(*data8++) << 8;
				crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
				crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
				crc ^= (uint16_t)(*data8++) << 8;
				crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
				crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
				crc ^= (uint16_t)(*data8++) << 8;
				crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
				crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
				crc ^= (uint16_t)(*data8++) << 8;
				crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
				crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
			#else
				crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
				crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
				crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
				crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
			#endif
			n -= 4;
		}

		while (n--)
		{
			#ifdef CRC_TABLE_4
				crc ^= (uint16_t)(*data8++) << 8;
				crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
				crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
			#else
				crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
			#endif
		}

		return crc;
	}

#else
	// bit-bang

	uint16_t FASTCALL CRC16(uint16_t crc, const uint8_t data)
	{	// bit bang (no table)
		unsigned int i;
		crc ^= data;
		for (i = 8; i > 0; i--)
			crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY16 : crc << 1;
		return crc;
	}

	uint16_t FASTCALL CRC16_block(uint16_t crc, const void *data, unsigned int n)
	{
		const uint8_t *data8 = (uint8_t *)data;
		while (n--)
			crc = CRC16(crc, *data8++);
		return crc;
	}

#endif
