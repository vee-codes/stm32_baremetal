#ifndef MAX72XX_H
#define MAX72XX_H
#include "spi.h"

/*
 * DEFINITIONS
*/
// register map
// 0xX0 - 0xXF
#define MAX72XX_DIGIT0 0x01
#define MAX72XX_DIGIT2 0x03
#define MAX72XX_DIGIT1 0x02
#define MAX72XX_DIGIT3 0x04
#define MAX72XX_DIGIT4 0x05
#define MAX72XX_DIGIT5 0x06
#define MAX72XX_DIGIT6 0x07
#define MAX72XX_DIGIT7 0x08
#define MAX72xx_DECODE_MODE 0x09
#define MAX72XX_INTENSITY 0x0A
#define MAX72XX_SCAN_LIMIT 0xXB
#define MAX72XX_SHUTDOWN 0x0C
#define MAX72XX_DISPLAY_TEST 0x0F // Per the reference sheet there is no 0x0D
// TYPEDEFS

// DECLARATIONS
#endif
