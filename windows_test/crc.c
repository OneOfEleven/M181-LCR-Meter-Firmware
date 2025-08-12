
// 1o11

#include "crc.h"

// ***********************************************
// 16-bit

// CCITT polynomial x^16 + x^12 + x^5 + 1
#define CRC_POLY16_FWD  0x1021
//#define CRC_POLY16_REV   0x8408

#ifdef USE_CRC_TABLES

	#ifdef USE_CRC_FLASH

		static const uint16_t CRC16_TABLE[] =
		{	// 0x1021
			0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
			0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,

			#ifndef CRC_TABLE_4
				0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
				0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
				0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
				0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
				0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
				0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
				0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
				0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
				0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
				0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
				0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
				0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
				0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
				0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,

				0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
				0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
				0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
				0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
				0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
				0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
				0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
				0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
				0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
				0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
				0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
				0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
				0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
				0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
				0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
				0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
			#endif
		};

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

			for (i = 0; i < (sizeof(CRC16_TABLE) / sizeof(CRC16_TABLE[0])); i++)
			{
				unsigned int k;
				uint16_t crc = i << 8;
//				uint16_t crc = i;
				for (k = 8; k > 0; k--)
					crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY16_FWD : crc << 1;
//					crc = (crc &     1u) ? (crc >> 1) ^ CRC_POLY16_REV : crc >> 1;
				CRC16_TABLE[i] = crc;
			}

			crc16_table_created = 1;
		}

	#endif

	uint16_t FASTCALL CRC16(uint16_t crc, const uint8_t data)
	{
		#ifndef USE_CRC_FLASH
			if (!crc16_table_created)
				make_CRC16_table();
		#endif

		#ifdef CRC_TABLE_4
			crc ^= (uint16_t)data << 8;
			crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
			crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
//			crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
//			crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
			return crc;
		#else
			return (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ data];
//			return (crc >> 8) ^ CRC16_TABLE[(crc & 0xff) ^ data];
		#endif
	}

	uint16_t FASTCALL CRC16_block(uint16_t crc, const void *data, unsigned int n)
	{
		const uint8_t *data8 = (uint8_t *)data;

		#ifndef USE_CRC_FLASH
			if (!crc16_table_created)
				make_CRC16_table();
		#endif

		while (n >= 4)
		{	// loop unroll
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
//				crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
//				crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
			#else
				crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
//				crc = (crc >> 8) ^ CRC16_TABLE[(crc & 0xff) ^ *data8++];
			#endif
		}

		return crc;
	}

#else
	// bit-bang

	uint16_t FASTCALL CRC16(uint16_t crc, const uint8_t data)
	{	// bit bang (no table)
		unsigned int i;
		crc ^= (uint16_t)data << 8;
//		crc ^= data;
		for (i = 8; i > 0; i--)
			crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY16_FWD : crc << 1;
//			crc = (crc &     1u) ? (crc >> 1) ^ CRC_POLY16_REV : crc >> 1;
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

// ***********************************************
// 32-bit

/*

// CCITT32 polynomial x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
#define CRC_POLY32_FWD       0x04C11DB7        // STM32 uses this one
//#define CRC_POLY32_REV       0xEDB88320

#ifdef USE_CRC_TABLES

	#ifdef USE_CRC_FLASH

		static const uint32_t CRC16_TABLE[] =
		{	// 0x04C11DB7
			0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
			0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61, 0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,

			#ifndef CRC_TABLE_4
				0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
				0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
				0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039, 0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
				0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
				0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
				0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1, 0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
				0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
				0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,
				0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde, 0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
				0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
				0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
				0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6, 0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
				0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
				0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,

				0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637, 0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
				0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
				0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
				0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff, 0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
				0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
				0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
				0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7, 0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
				0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
				0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
				0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8, 0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
				0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
				0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
				0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0, 0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
				0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
				0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
				0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668, 0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
			#endif
		};

	#else

		uint8_t crc32_table_created = 0;

		#ifdef CRC_TABLE_4
			uint16_t CRC32_TABLE[16];
		#else
			uint16_t CRC32_TABLE[256];
		#endif

		void FASTCALL make_CRC32_table(void)
		{
			uint32_t i;

			if (crc32_table_created)
				return;

			for (i = 0; i < (sizeof(CRC32_TABLE) / sizeof(CRC32_TABLE[0])); i++)
			{
				unsigned int k;
				uint32_t crc = i << 24;
//				uint32_t crc = i;
				for (k = 8; k > 0; k--)
					crc = (crc & 0x80000000) ? (crc << 1) ^ CRC_POLY32_FWD : crc << 1;
//					crc = (crc &         1u) ? (crc >> 1) ^ CRC_POLY32_REV : crc >> 1;
				CRC32_TABLE[i] = crc;
			}

			crc32_table_created = 1;
		}

	#endif

	uint32_t FASTCALL CRC32(uint32_t crc, const uint8_t data)
	{
		#ifndef USE_CRC_FLASH
			if (!crc32_table_created)
				make_CRC32_table();
		#endif

		#ifdef CRC_TABLE_4
			crc ^= (uint32_t)data << 24;
			crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
			crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
//			crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
//			crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
			return crc;
		#else
			return (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ data];
//			return (crc >> 8) ^ CRC32_TABLE[(crc & 0xff) ^ data];
		#endif
	}

	uint32_t FASTCALL CRC32_block(uint32_t crc, const void *data, unsigned int n)
	{
		const uint8_t *data8 = (uint8_t *)data;

		#ifndef USE_CRC_FLASH
			if (!crc32_table_created)
				make_CRC32_table();
		#endif

		while (n >= 4)
		{	// loop unroll
			#ifdef CRC_TABLE_4
				crc ^= (uint32_t)(*data8++) << 24;
				crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
				crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
				crc ^= (uint32_t)(*data8++) << 24;
				crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
				crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
				crc ^= (uint32_t)(*data8++) << 24;
				crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
				crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
				crc ^= (uint32_t)(*data8++) << 24;
				crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
				crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
			#else
				crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ *data8++];
				crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ *data8++];
				crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ *data8++];
				crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ *data8++];
			#endif
			n -= 4;
		}

		while (n--)
		{
			#ifdef CRC_TABLE_4
				crc ^= (uint32_t)(*data8++) << 24;
				crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
				crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
//				crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
//				crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
			#else
				crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ *data8++];
//				crc = (crc >> 8) ^ CRC32_TABLE[(crc & 0xff) ^ *data8++];
			#endif
		}

		return crc;
	}

#else
	// bit-bang

	uint32_t FASTCALL CRC32(uint32_t crc, const uint8_t data)
	{	// bit bang (no table)
		unsigned int i;
		crc ^= (uint32_t)data << 24;
//		crc ^= data;
		for (i = 8; i > 0; i--)
			crc = (crc & 0x80000000) ? (crc << 1) ^ CRC_POLY32_FWD : crc << 1;
//			crc = (crc &         1u) ? (crc >> 1) ^ CRC_POLY32_REV : crc >> 1;
		return crc;
	}

	uint32_t FASTCALL CRC32_block(uint32_t crc, const void *data, unsigned int n)
	{
		const uint8_t *data8 = (uint8_t *)data;
		while (n--)
			crc = CRC32(crc, *data8++);
		return crc;
	}

#endif

*/