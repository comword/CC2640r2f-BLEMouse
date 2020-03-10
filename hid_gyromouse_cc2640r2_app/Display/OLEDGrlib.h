/*
 * OLEDGrlib.h
 *
 *      Author: henorvell
 */

#ifndef DISPLAY_OLEDGRLIB_H_
#define DISPLAY_OLEDGRLIB_H_

#include <ti/drivers/PIN.h>
#include <ti/drivers/I2C.h>

// Define LCD Screen Orientation Here
#define LANDSCAPE
#define MAX_PALETTE_COLORS                2

#define SHARP_BLACK                       0x00
#define SHARP_WHITE                       0xFF

extern const Graphics_Display_Functions g_sharpFxns;

extern void OLEDGrLib_init(I2C_Handle hI2C, PIN_Handle hPin);

#endif /* DISPLAY_OLEDGRLIB_H_ */
