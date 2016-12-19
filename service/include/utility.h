/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:43:01
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-12 00:39:47
*/

#ifndef __ULTILITY_H__
#define __ULTILITY_H__

/* Helper function */

/* Macro to convert the MSB/LSB of a 16-bit value to a 8-bit value*/
#define GET_16BIT_MSB( x )        ( (uint8_t)( ( ( (uint16_t)(x) ) >> 8 ) & 0xFF ) )
#define GET_16BIT_LSB( x )        ( (uint8_t)(   ( (uint16_t)(x) )        & 0xFF ) )

/* Macro to write the 16-bit value into an 8-bit buffer */
#define BUF_SET_16BIT( buf, value )                             \
    do {                                                        \
        ((uint8_t *)(buf))[ 0 ] = GET_16BIT_MSB(value);         \
        ((uint8_t *)(buf))[ 1 ] = GET_16BIT_LSB(value);         \
    } while ( 0 )

/* macro to read a 16-bit value from an 8-bit buffer */
#define BUF_GET_16BIT( buf )                                    \
    ( ( ((uint16_t)( ((uint8_t *)(buf))[ 0 ] ) ) << 8   )       \
    | ( ((uint16_t)( ((uint8_t *)(buf))[ 1 ] ) ) & 0xFF )       \
    )

/* Convert String to Integer*/
int to_int(char *s);

/* Convert an unsigned int to signed int*/
int16_t unsignedToSigned(uint16_t val);

/* Use this function to convert a 16-bit number into little endian. */
uint16_t u16ToLittleEndian(uint16_t u16input );

/* Use this function to convert a 32-bit number into little endian. */
uint32_t u32ToLittleEndian(uint32_t u32input );

#endif
