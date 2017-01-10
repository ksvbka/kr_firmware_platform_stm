/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:43:01
* @Last Modified by:   Kienltb
* @Last Modified time: 2017-01-09 16:22:17
*/

#ifndef __ULTILITY_H__
#define __ULTILITY_H__

/* Helper function */

/* Convert to 16bit  */
#define CONVERT_TO_16BIT(MSB, LSB)  (((uint16_t)(MSB) << 8) | (uint16_t)(LSB))

/* Abs function*/
#define ABS(x) (x < 0 ? -x : x)

/* Convert String to Integer*/
int to_int(char *s);

/* Check the system is big endian for little endian. NOTE: need compiler support C99*/
#define IS_BIG_ENDIAN (!*(unsigned char *)&(unsigned short){1})

/* Convert an unsigned int to signed int*/
int16_t unsignedToSigned(uint16_t val);

/* Use this function to convert a 16-bit number into little endian. */
uint16_t u16ToLittleEndian(uint16_t u16input );

/* Use this function to convert a 32-bit number into little endian. */
uint32_t u32ToLittleEndian(uint32_t u32input );

/* Convert a float into 4byte and store to buffer */
void float_to_buff(float data, char* buffer);

/* Convert a 4byte in buffer to float */
float buffer_to_float(char* buffer);

#endif
