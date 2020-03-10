/*
 * OLEDriver.c
 *      Author: henorvell
 */

#include "Display/OLEDisplay.h"

// Timeout of semaphore that controls exclusive to the LCD (infinite)
#define ACCESS_TIMEOUT    SemaphoreP_WAIT_FOREVER

const Display_FxnTable DisplayOLED_fxnTable = {
    DisplayOLED_init,
    DisplayOLED_open,
    DisplayOLED_clear,
    DisplayOLED_clearLines,
    DisplayOLED_vprintf,
    DisplayOLED_close,
    DisplayOLED_control,
    DisplayOLED_getType,
};

void DisplayOLED_init(Display_Handle handle)
{
}

Display_Handle DisplayOLED_open(Display_Handle handle, Display_Params * params)
{
    return 0;
}

void DisplayOLED_clear(Display_Handle hDisplay)
{
    DisplayOLED_Object *object = (DisplayOLED_Object *)hDisplay->object;
    if (object->hPins == NULL)
    {
        return;
    }

    // Grab LCD
    if (SemaphoreP_pend((SemaphoreP_Handle) & object->semLCD, ACCESS_TIMEOUT) == SemaphoreP_OK)
    {
        Graphics_clearDisplay(&object->g_sContext);
        Graphics_flushBuffer(&object->g_sContext);
        // Release LCD
        SemaphoreP_post((SemaphoreP_Handle) & object->semLCD);
    }
}

void DisplayOLED_clearLines(Display_Handle hDisplay, uint8_t lineTo, uint8_t lineFrom)
{
    DisplayOLED_Object *object = (DisplayOLED_Object  *)hDisplay->object;
    if (lineTo <= lineFrom)
    {
        lineTo = lineFrom;
    }

    Graphics_Rectangle rect = {
        .xMin = 0,
        .xMax = object->g_sContext.clipRegion.xMax,
        .yMin = lineFrom * object->g_sContext.font->height,
        .yMax = (lineTo + 1) * object->g_sContext.font->height - 1,
    };

    Graphics_setForegroundColor(&object->g_sContext, object->displayColor.bg);
    Graphics_fillRectangle(&object->g_sContext, &rect);
    Graphics_setForegroundColor(&object->g_sContext, object->displayColor.fg);
    Graphics_flushBuffer(&object->g_sContext);
}

void DisplayOLED_vprintf(Display_Handle hDisplay, uint8_t line, uint8_t column, char *fmt, va_list va)
{
    DisplayOLED_Object *object = (DisplayOLED_Object  *)hDisplay->object;
    uint8_t xp, yp, clearStartX, clearEndX;
    char    dispStr[23];
    if (object->hPins == NULL)
    {
        return;
    }
    // Grab LCD
    if (SemaphoreP_pend((SemaphoreP_Handle) & object->semLCD, ACCESS_TIMEOUT) == SemaphoreP_OK)
    {
        xp          = column * object->g_sContext.font->maxWidth + 1;
        yp          = line * object->g_sContext.font->height + 0;
        clearStartX = clearEndX = xp;

        switch (object->lineClearMode)
        {
        case DISPLAY_CLEAR_LEFT:
            clearStartX = 0;
            break;
        case DISPLAY_CLEAR_RIGHT:
            clearEndX = object->g_sContext.clipRegion.xMax;
            break;
        case DISPLAY_CLEAR_BOTH:
            clearStartX = 0;
            clearEndX   = object->g_sContext.clipRegion.xMax;
            break;
        case DISPLAY_CLEAR_NONE:
        default:
            break;
        }

        if (clearStartX != clearEndX)
        {
            Graphics_Rectangle rect = {
                .xMin = clearStartX,
                .xMax = clearEndX,
                .yMin = yp,
                .yMax = yp + object->g_sContext.font->height - 1,
            };

            Graphics_setForegroundColor(&object->g_sContext, object->displayColor.bg);
            Graphics_fillRectangle(&object->g_sContext, &rect);
            Graphics_setForegroundColor(&object->g_sContext, object->displayColor.fg);
        }

        SystemP_vsnprintf(dispStr, sizeof(dispStr), fmt, va);

        // Draw a text on the display
        Graphics_drawString(&object->g_sContext,
                           (int8_t *)dispStr,
                           AUTO_STRING_LENGTH,
                           xp,
                           yp,
                           OPAQUE_TEXT);

        Graphics_flushBuffer(&object->g_sContext);

        // Release LCD
        SemaphoreP_post((SemaphoreP_Handle) & object->semLCD);
    }
}

void DisplayOLED_close(Display_Handle handle)
{

}

int DisplayOLED_control(Display_Handle handle, unsigned int cmd, void *arg)
{
    return 0;
}

unsigned int DisplayOLED_getType(void)
{
    return Display_Type_LCD | Display_Type_GRLIB;
}
