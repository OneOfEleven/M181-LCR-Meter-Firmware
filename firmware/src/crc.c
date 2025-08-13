
// 1o11

#include "crc.h"

// ***************************************
/*
inline uint8_t FASTCALL bit_rev8(uint8_t n)
{
	n = ((n >> 1) & 0x55u) | ((n << 1) & 0xAAu);
	n = ((n >> 2) & 0x33u) | ((n << 2) & 0xCCu);
	n = ((n >> 4) & 0x0Fu) | ((n << 4) & 0xF0u);
	return n;
}

inline uint16_t FASTCALL bit_rev16(uint16_t n)
{	// untested
	n = ((n >> 1) & 0x5555u) | ((n << 1) & 0xAAAAu);
	n = ((n >> 2) & 0x3333u) | ((n << 2) & 0xCCCCu);
	n = ((n >> 4) & 0x0F0Fu) | ((n << 4) & 0xF0F0u);
	n = ((n >> 8) & 0x00FFu) | ((n << 8) & 0xFF00u);
   return n;
}

inline uint32_t FASTCALL bit_rev32(uint32_t n)
{
	n = ((n >>  1) & 0x55555555u) | ((n <<  1) & 0xAAAAAAAAu);
	n = ((n >>  2) & 0x33333333u) | ((n <<  2) & 0xCCCCCCCCu);
	n = ((n >>  4) & 0x0F0F0F0Fu) | ((n <<  4) & 0xF0F0F0F0u);
	n = ((n >>  8) & 0x00FF00FFu) | ((n <<  8) & 0xFF00FF00u);
	n = ((n >> 16) & 0x0000FFFFu) | ((n << 16) & 0xFFFF0000u);
	return n;
}

inline uint64_t FASTCALL bit_rev64(uint64_t n)
{
	n = ((n >>  1) & 0x5555555555555555u) | ((n <<  1) & 0xAAAAAAAAAAAAAAAAu);
	n = ((n >>  2) & 0x3333333333333333u) | ((n <<  2) & 0xCCCCCCCCCCCCCCCCu);
	n = ((n >>  4) & 0x0F0F0F0F0F0F0F0Fu) | ((n <<  4) & 0xF0F0F0F0F0F0F0F0u);
	n = ((n >>  8) & 0x00FF00FF00FF00FFu) | ((n <<  8) & 0xFF00FF00FF00FF00u);
	n = ((n >> 16) & 0x0000FFFF0000FFFFu) | ((n << 16) & 0xFFFF0000FFFF0000u);
	n = ((n >> 32) & 0x00000000FFFFFFFFu) | ((n << 32) & 0xFFFFFFFF00000000u);
	return n;
}
*/
// ***************************************

#if defined(CRC_FWD_16) || defined(CRC_REV_16)
	// 16-bit

	// CCITT polynomial x^16 + x^12 + x^5 + 1
	#define CRC_POLY16_FWD   0x1021     // MS-1st .. 1 0001 0000 0010 0001
	#define CRC_POLY16_REV   0x8408     // LS-1st .. 1000 0100 0000 1000 1

	#ifdef USE_CRC_TABLES

		#ifdef USE_CRC_FLASH
			// table is located in FLASH

			#if defined(CRC_FWD_16)
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
			#elif defined(CRC_REV_16)
				static const uint16_t CRC16_TABLE[] =
				{	// 0x8408
					0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
					0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,

					#ifndef CRC_TABLE_4
						0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
						0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
						0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
						0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
						0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
						0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
						0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
						0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
						0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
						0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
						0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
						0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
						0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
						0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,

						0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
						0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
						0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
						0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
						0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
						0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
						0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
						0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
						0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
						0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
						0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
						0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
						0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
						0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
						0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
						0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
					#endif
				};
			#endif

		#else
			// table is located in RAM

			uint8_t crc16_table_created = 0;

			#pragma pack(push, 1)
				#ifdef CRC_TABLE_4
					uint16_t CRC16_TABLE[16];             // nibble table
				#else
					uint16_t CRC16_TABLE[256];            // full table
				#endif
			#pragma pack(pop)

			void FASTCALL make_CRC16_table(void)
			{
				uint16_t i;

				if (crc16_table_created)
					return;

				#if defined(CRC_FWD_16)
					for (i = 0; i < (sizeof(CRC16_TABLE) / sizeof(CRC16_TABLE[0])); i++)
					{
						unsigned int k;
						uint16_t crc = i << 8;
						for (k = 8; k > 0; k--)
							crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY16_FWD : crc << 1;
						CRC16_TABLE[i] = crc;
					}
				#elif defined(CRC_REV_16)
					for (i = 0; i < (sizeof(CRC16_TABLE) / sizeof(CRC16_TABLE[0])); i++)
					{
						unsigned int k;
						uint16_t crc = i;
						for (k = 8; k > 0; k--)
							crc = (crc &     1u) ? (crc >> 1) ^ CRC_POLY16_REV : crc >> 1;
						CRC16_TABLE[i] = crc;
					}
				#endif

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
				#if defined(CRC_FWD_16)
					crc ^= (uint16_t)data << 8;
					crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
					crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
				#elif defined(CRC_REV_16)
					crc ^= data;
					crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
					crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
				#endif
				return crc;
			#else
				#if defined(CRC_FWD_16)
					return (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ data];
				#elif defined(CRC_REV_16)
					return (crc >> 8) ^ CRC16_TABLE[(crc & 0xff) ^ data];
				#endif
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
					#if defined(CRC_FWD_16)
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
					#elif defined(CRC_REV_16)
						crc ^= *data8++;
						crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
						crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
						crc ^= *data8++;
						crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
						crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
						crc ^= *data8++;
						crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
						crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
						crc ^= *data8++;
						crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
						crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
					#endif
				#else
					#if defined(CRC_FWD_16)
						crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
						crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
						crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
						crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
					#elif defined(CRC_REV_16)
						crc = (crc >> 8) ^ CRC16_TABLE[(crc & 0xff) ^ *data8++];
						crc = (crc >> 8) ^ CRC16_TABLE[(crc & 0xff) ^ *data8++];
						crc = (crc >> 8) ^ CRC16_TABLE[(crc & 0xff) ^ *data8++];
						crc = (crc >> 8) ^ CRC16_TABLE[(crc & 0xff) ^ *data8++];
					#endif
				#endif
				n -= 4;
			}

			while (n--)
			{
				#ifdef CRC_TABLE_4
					#if defined(CRC_FWD_16)
						crc ^= (uint16_t)(*data8++) << 8;
						crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
						crc = (crc << 4) ^ CRC16_TABLE[crc >> 12];
					#elif defined(CRC_REV_16)
						crc ^= *data8++;
						crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
						crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x0f];
					#endif
				#else
					#if defined(CRC_FWD_16)
						crc = (crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ *data8++];
					#elif defined(CRC_REV_16)
						crc = (crc >> 8) ^ CRC16_TABLE[(crc & 0xff) ^ *data8++];
					#endif
				#endif
			}

			return crc;
		}

	#else
		// bit-bang

		uint16_t FASTCALL CRC16(uint16_t crc, const uint8_t data)
		{	// bit bang (no table)
			unsigned int i;
			#if defined(CRC_FWD_16)
				crc ^= (uint16_t)data << 8;
				for (i = 8; i > 0; i--)
					crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY16_FWD : crc << 1;
			#elif defined(CRC_REV_16)
				crc ^= data;
				for (i = 8; i > 0; i--)
					crc = (crc &     1u) ? (crc >> 1) ^ CRC_POLY16_REV : crc >> 1;
			#endif
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
#endif

#if defined(CRC_FWD_32) || defined(CRC_REV_32)
	// 32-bit

	// CCITT32 polynomial x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
	#define CRC_POLY32_FWD     0x04C11DB7        // MS-1st .. 1 0000 0100 1100 0001 0001 1101 1011 0111 .. STM32 uses this one
	#define CRC_POLY32_REV     0xEDB88320        // LS-1st .. 1110 1101 1011 1000 1000 0011 0010 0000 1

	#ifdef USE_CRC_TABLES

		#ifdef USE_CRC_FLASH
			// table is located in FLASH

			#if defined(CRC_FWD_32)
				static const uint32_t CRC32_TABLE[] =
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
			#elif defined(CRC_REV_32)
				static const uint32_t CRC32_TABLE[] =
				{	// 0xEDB88320
					0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
					0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,

					#ifndef CRC_TABLE_4
						0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
						0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
						0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
						0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
						0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
						0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
						0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
						0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
						0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
						0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
						0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
						0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
						0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
						0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,

						0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
						0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
						0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
						0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
						0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
						0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
						0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
						0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
						0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
						0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
						0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
						0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
						0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
						0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
						0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
						0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
					#endif
				};
			#endif

		#else
			// table is located in RAM

			uint8_t crc32_table_created = 0;

			#pragma pack(push, 1)
				#ifdef CRC_TABLE_4
					uint32_t CRC32_TABLE[16];          // nibble table
				#else
					uint32_t CRC32_TABLE[256];         // full table
				#endif
			#pragma pack(pop)

			void FASTCALL make_CRC32_table(void)
			{
				uint32_t i;

				if (crc32_table_created)
					return;

				#if defined(CRC_FWD_32)
					for (i = 0; i < (sizeof(CRC32_TABLE) / sizeof(CRC32_TABLE[0])); i++)
					{
						unsigned int k;
						uint32_t crc = i << 24;
//						uint32_t crc = __RBIT(i);
						for (k = 8; k > 0; k--)
							crc = (crc & 0x80000000) ? (crc << 1) ^ CRC_POLY32_FWD : crc << 1;
						CRC32_TABLE[i] = crc;
					}
				#elif defined(CRC_REV_32)
					for (i = 0; i < (sizeof(CRC32_TABLE) / sizeof(CRC32_TABLE[0])); i++)
					{
						unsigned int k;
						uint32_t crc = i;
						for (k = 8; k > 0; k--)
							crc = (crc &         1u) ? (crc >> 1) ^ CRC_POLY32_REV : crc >> 1;
						CRC32_TABLE[i] = crc;
					}
				#endif

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
				#if defined(CRC_FWD_32)
					crc ^= (uint32_t)data << 24;
//					crc ^= __RBIT(data);
					crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
					crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
				#elif defined(CRC_REV_32)
					crc ^= data;
					crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
					crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
				#endif
				return crc;
			#else
				#if defined(CRC_FWD_32)
					return (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ data];
//					return (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ __RBIT(data)];
				#elif defined(CRC_REV_32)
					return (crc >> 8) ^ CRC32_TABLE[(crc & 0xff) ^ data];
				#endif
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
					#if defined(CRC_FWD_32)
						crc ^= (uint32_t)*data8++ << 24;
//						crc ^= __RBIT(~*data8++);
						crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
						crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
						crc ^= (uint32_t)*data8++ << 24;
//						crc ^= __RBIT(~*data8++);
						crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
						crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
						crc ^= (uint32_t)*data8++ << 24;
//						crc ^= __RBIT(~*data8++);
						crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
						crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
						crc ^= (uint32_t)*data8++ << 24;
//						crc ^= __RBIT(~*data8++);
						crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
						crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
					#elif defined(CRC_REV_32)
						crc ^= *data8++;
						crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
						crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
						crc ^= *data8++;
						crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
						crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
						crc ^= *data8++;
						crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
						crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
						crc ^= *data8++;
						crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
						crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
					#endif
				#else
					#if defined(CRC_FWD_32)
						crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ *data8++];
						crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ *data8++];
						crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ *data8++];
						crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ *data8++];
					#elif defined(CRC_REV_32)
						crc = (crc >> 8) ^ CRC32_TABLE[(crc & 0xff) ^ *data8++];
						crc = (crc >> 8) ^ CRC32_TABLE[(crc & 0xff) ^ *data8++];
						crc = (crc >> 8) ^ CRC32_TABLE[(crc & 0xff) ^ *data8++];
						crc = (crc >> 8) ^ CRC32_TABLE[(crc & 0xff) ^ *data8++];
					#endif
				#endif
				n -= 4;
			}

			while (n--)
			{
				#ifdef CRC_TABLE_4
					#if defined(CRC_FWD_32)
						crc ^= (uint32_t)*data8++ << 24;
//						crc ^= __RBIT(~*data8++);
						crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
						crc = (crc << 4) ^ CRC32_TABLE[crc >> 28];
					#elif defined(CRC_REV_32)
						crc ^= *data8++;
						crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
						crc = (crc >> 4) ^ CRC32_TABLE[crc & 0x0f];
					#endif
				#else
					#if defined(CRC_FWD_32)
						crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ *data8++];
//						crc = (crc << 8) ^ CRC32_TABLE[(crc >> 24) ^ __RBIT(~*data8++)];
					#elif defined(CRC_REV_32)
						crc = (crc >> 8) ^ CRC32_TABLE[(crc & 0xff) ^ *data8++];
					#endif
				#endif
			}

			//crc = __RBIT(crc) ^ 0xffffffff;

			return crc;
		}

	#else
		// bit-bang

		uint32_t FASTCALL CRC32(uint32_t crc, const uint8_t data)
		{	// bit bang (no table)
			unsigned int i;
			#if defined(CRC_FWD_32)
				crc ^= (uint32_t)data << 24;
//				crc ^= __RBIT(data);
				for (i = 8; i > 0; i--)
					crc = (crc & 0x80000000) ? (crc << 1) ^ CRC_POLY32_FWD : crc << 1;
			#elif defined(CRC_REV_32)
				crc ^= data;
				for (i = 8; i > 0; i--)
					crc = (crc &         1u) ? (crc >> 1) ^ CRC_POLY32_REV : crc >> 1;
			#endif
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
#endif
