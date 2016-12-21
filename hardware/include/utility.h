/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:43:01
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-12 00:39:47
*/

#ifndef __ULTILITY_H__
#define __ULTILITY_H__

/* Helper function */

/* Convert String to Integer*/
int to_int(char *s);

/* Convert an unsigned int to signed int*/
int16_t unsignedToSigned(uint16_t val);

/* Use this function to convert a 16-bit number into little endian. */
uint16_t u16ToLittleEndian(uint16_t u16input );

/* Use this function to convert a 32-bit number into little endian. */
uint32_t u32ToLittleEndian(uint32_t u32input );

#endif
