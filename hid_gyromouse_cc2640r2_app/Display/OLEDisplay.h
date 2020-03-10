/*
 * OLEDriver.h
 *      Author: henorvell
 */

#ifndef DISPLAY_OLEDISPLAY_H_
#define DISPLAY_OLEDISPLAY_H_

#include <ti/drivers/dpl/SemaphoreP.h>

#include <ti/drivers/PIN.h>
#include <ti/drivers/I2C.h>

#include <ti/display/Display.h>
#include <ti/grlib/grlib.h>

#include <stdint.h>

#define OLED_CMD_SET_COLORS  DISPLAY_CMD_RESERVED + 0

extern const Display_FxnTable DisplayOLED_fxnTable;

typedef struct
{
    uint32_t fg;
    uint32_t bg;
} OLEDColor_t;

typedef struct OLED_HWAttrs
{
    uint32_t I2CIndex; // Board only has I2C0
//    PIN_Id   csPin;
//    PIN_Id   extcominPin;
//    PIN_Id   powerPin;
//    PIN_Id   enablePin;
    PIN_Id SDAPin;
    PIN_Id SCLPin;
    uint16_t pixelHeight;
    uint16_t pixelWidth;
    void    *displayBuf;
} OLED_HWAttrs;

typedef uint8_t OLED_Buf_128x64[128 * 64 / 8];

typedef struct DisplayOLED_Object
{
    Graphics_Context      g_sContext;
    I2C_Handle            hI2C;
    PIN_State             pinState;
    PIN_Handle            hPins;
    Display_LineClearMode lineClearMode;
    Graphics_Display      g_sDisplay;
    OLEDColor_t   displayColor;
    SemaphoreP_Struct     semLCD;
    uint8_t              *displayBuffer;
} DisplayOLED_Object, *DisplayOLED_Handle;

void DisplayOLED_init(Display_Handle handle);
Display_Handle DisplayOLED_open(Display_Handle,
                                 Display_Params * params);
void DisplayOLED_clear(Display_Handle handle);
void DisplayOLED_clearLines(Display_Handle handle,
                             uint8_t fromLine,
                             uint8_t toLine);
void DisplayOLED_vprintf(Display_Handle handle, uint8_t line,
                          uint8_t column, char *fmt, va_list va);
void DisplayOLED_close(Display_Handle);
int DisplayOLED_control(Display_Handle handle, unsigned int cmd, void *arg);
unsigned int DisplayOLED_getType(void);

#endif /* DISPLAY_OLEDISPLAY_H_ */
