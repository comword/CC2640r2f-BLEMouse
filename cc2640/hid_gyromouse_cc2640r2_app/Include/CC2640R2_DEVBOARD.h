/*
 * CC2640R2_MOUSEBOARD.h
 *
 *  Created on: Jul 14, 2018
 *      Author: henorvell
 */

#ifndef _CC2640R2_MOUSEBOARD_H_
#define _CC2640R2_MOUSEBOARD_H_
#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <ti/drivers/PIN.h>
#include <ti/devices/cc26x0r2/driverlib/ioc.h>

/* Externs */
extern const PIN_Config BoardGpioInitTable[];

/* Defines */
#ifndef CC2640R2_MOUSEBOARD
  #define CC2640R2_MOUSEBOARD
#endif

/*
 *  ============================================================================
 *  RF Front End and Bias configuration symbols for TI reference designs and
 *  kits. This symbol sets the RF Front End configuration in ble_user_config.h
 *  and selects the appropriate PA table in ble_user_config.c.
 *  Other configurations can be used by editing these files.
 *
 *  Define only one symbol:
 *  CC2650EM_7ID    - Differential RF and internal biasing
                      (default for CC2640R2 LaunchPad)
 *  CC2650EM_5XD    – Differential RF and external biasing
 *  CC2650EM_4XS    – Single-ended RF on RF-P and external biasing
 *  CC2640R2DK_CXS  - WCSP: Single-ended RF on RF-N and external biasing
 *                    (Note that the WCSP is only tested and characterized for
 *                     single ended configuration, and it has a WCSP-specific
 *                     PA table)
 *
 *  Note: CC2650EM_xxx reference designs apply to all CC26xx devices.
 *  ==========================================================================
 */
#define CC2650EM_7ID

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                  <pin mapping>
 */

/* Analog Capable DIOs */
#define CC2640R2_LAUNCHXL_DIO23_ANALOG          IOID_23
#define CC2640R2_LAUNCHXL_DIO24_ANALOG          IOID_24
#define CC2640R2_LAUNCHXL_DIO25_ANALOG          IOID_25
#define CC2640R2_LAUNCHXL_DIO26_ANALOG          IOID_26
#define CC2640R2_LAUNCHXL_DIO27_ANALOG          IOID_27
#define CC2640R2_LAUNCHXL_DIO28_ANALOG          IOID_28
#define CC2640R2_LAUNCHXL_DIO29_ANALOG          IOID_29
#define CC2640R2_LAUNCHXL_DIO30_ANALOG          IOID_30

/* Digital IOs */
#define CC2640R2_LAUNCHXL_DIO0                  IOID_0
#define CC2640R2_LAUNCHXL_DIO1_RFSW             IOID_1
#define CC2640R2_LAUNCHXL_DIO12                 IOID_12
#define CC2640R2_LAUNCHXL_DIO15                 IOID_15
#define CC2640R2_LAUNCHXL_DIO16_TDO             IOID_16
#define CC2640R2_LAUNCHXL_DIO17_TDI             IOID_17
#define CC2640R2_LAUNCHXL_DIO21                 IOID_21
#define CC2640R2_LAUNCHXL_DIO22                 IOID_22

/* Discrete Inputs */
#define CC2640R2_LAUNCHXL_PIN_RBTN              IOID_19
#define CC2640R2_LAUNCHXL_PIN_CBTN              IOID_20
#define CC2640R2_LAUNCHXL_PIN_LBTN              IOID_21

/* I2C */
#define CC2640R2_LAUNCHXL_I2C0_SCL0             IOID_13
#define CC2640R2_LAUNCHXL_I2C0_SDA0             IOID_12

/* LCD (430BOOST - Sharp96 Rev 1.1) */
#define CC2640R2_LCD_SDA                        IOID_12
#define CC2640R2_LCD_SCL                        IOID_13
#define CC2640R2_LCD_RST                        IOID_14
//#define CC2640R2_LAUNCHXL_LCD_CS                IOID_24 /* SPI chip select */
//#define CC2640R2_LAUNCHXL_LCD_EXTCOMIN          IOID_12 /* External COM inversion */
//#define CC2640R2_LAUNCHXL_LCD_ENABLE            IOID_22 /* LCD enable */
//#define CC2640R2_LAUNCHXL_LCD_POWER             IOID_23 /* LCD power control */
//#define CC2640R2_LAUNCHXL_LCD_CS_ON             1
//#define CC2640R2_LAUNCHXL_LCD_CS_OFF            0

/* LEDs */
#define CC2640R2_LAUNCHXL_PIN_RLED              IOID_5
#define CC2640R2_LAUNCHXL_PIN_BLED              IOID_6

/* PWM Outputs */
#define CC2640R2_LAUNCHXL_PWMPIN0               CC2640R2_LAUNCHXL_PIN_RLED
#define CC2640R2_LAUNCHXL_PWMPIN1               CC2640R2_LAUNCHXL_PIN_BLED
#define CC2640R2_LAUNCHXL_PWMPIN2               PIN_UNASSIGNED
#define CC2640R2_LAUNCHXL_PWMPIN3               PIN_UNASSIGNED
#define CC2640R2_LAUNCHXL_PWMPIN4               PIN_UNASSIGNED
#define CC2640R2_LAUNCHXL_PWMPIN5               PIN_UNASSIGNED
#define CC2640R2_LAUNCHXL_PWMPIN6               PIN_UNASSIGNED
#define CC2640R2_LAUNCHXL_PWMPIN7               PIN_UNASSIGNED

///* SPI */
//#define CC2640R2_LAUNCHXL_SPI_FLASH_CS          IOID_20
//#define CC2640R2_LAUNCHXL_FLASH_CS_ON           0
//#define CC2640R2_LAUNCHXL_FLASH_CS_OFF          1

/* SPI Board */
//SPI0 works in SLAVE mode.
#define CC2640R2_LAUNCHXL_SPI0_MISO             IOID_8
#define CC2640R2_LAUNCHXL_SPI0_MOSI             IOID_9
#define CC2640R2_LAUNCHXL_SPI0_CLK              IOID_10
#define CC2640R2_LAUNCHXL_SPI0_CSN              IOID_11
#define CC2640R2_LAUNCHXL_SPI1_MISO             PIN_UNASSIGNED
#define CC2640R2_LAUNCHXL_SPI1_MOSI             PIN_UNASSIGNED
#define CC2640R2_LAUNCHXL_SPI1_CLK              PIN_UNASSIGNED
#define CC2640R2_LAUNCHXL_SPI1_CSN              PIN_UNASSIGNED

/* UART Board */
#define CC2640R2_LAUNCHXL_UART_RX               IOID_2
#define CC2640R2_LAUNCHXL_UART_TX               IOID_3
#define CC2640R2_LAUNCHXL_UART_CTS              PIN_UNASSIGNED
#define CC2640R2_LAUNCHXL_UART_RTS              PIN_UNASSIGNED

/* MPU Interrupt */
#define CC2640R2_MPU_INT                        IOID_18

/* Power status Pin */
#define CC2640R2_BAT_ADC                        IOID_24
#define CC2640R2_BAT_PGOOD                      IOID_25
#define CC2640R2_BAT_CHG                        IOID_26

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
void CC2640R2_LAUNCHXL_initGeneral(void);

/*!
 *  @brief  Turn off the external flash on LaunchPads
 *
 */
//void CC2640R2_LAUNCHXL_shutDownExtFlash(void);

/*!
 *  @def    CC2640R2_LAUNCHXL_ADCBufName
 *  @brief  Enum of ADCs
 */
typedef enum CC2640R2_LAUNCHXL_ADCBufName {
    CC2640R2_LAUNCHXL_ADCBUF0 = 0,

    CC2640R2_LAUNCHXL_ADCBUFCOUNT
} CC2640R2_LAUNCHXL_ADCBufName;

/*!
 *  @def    CC2640R2_LAUNCHXL_ADCBuf0SourceName
 *  @brief  Enum of ADCBuf channels
 */
typedef enum CC2640R2_LAUNCHXL_ADCBuf0ChannelName {
    CC2640R2_LAUNCHXL_ADCBUF0CHANNEL0 = 0,
    CC2640R2_LAUNCHXL_ADCBUF0CHANNEL1,
    CC2640R2_LAUNCHXL_ADCBUF0CHANNEL2,
    CC2640R2_LAUNCHXL_ADCBUF0CHANNEL3,
    CC2640R2_LAUNCHXL_ADCBUF0CHANNEL4,
    CC2640R2_LAUNCHXL_ADCBUF0CHANNEL5,
    CC2640R2_LAUNCHXL_ADCBUF0CHANNEL6,
    CC2640R2_LAUNCHXL_ADCBUF0CHANNEL7,
    CC2640R2_LAUNCHXL_ADCBUF0CHANNELVDDS,
    CC2640R2_LAUNCHXL_ADCBUF0CHANNELDCOUPL,
    CC2640R2_LAUNCHXL_ADCBUF0CHANNELVSS,

    CC2640R2_LAUNCHXL_ADCBUF0CHANNELCOUNT
} CC2640R2_LAUNCHXL_ADCBuf0ChannelName;

/*!
 *  @def    CC2640R2_LAUNCHXL_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum CC2640R2_LAUNCHXL_ADCName {
    CC2640R2_LAUNCHXL_ADC0 = 0,
    CC2640R2_LAUNCHXL_ADC1,
    CC2640R2_LAUNCHXL_ADC2,
    CC2640R2_LAUNCHXL_ADC3,
    CC2640R2_LAUNCHXL_ADC4,
    CC2640R2_LAUNCHXL_ADC5,
    CC2640R2_LAUNCHXL_ADC6,
    CC2640R2_LAUNCHXL_ADC7,
    CC2640R2_LAUNCHXL_ADCDCOUPL,
    CC2640R2_LAUNCHXL_ADCVSS,
    CC2640R2_LAUNCHXL_ADCVDDS,

    CC2640R2_LAUNCHXL_ADCCOUNT
} CC2640R2_LAUNCHXL_ADCName;

/*!
 *  @def    CC2640R2_LAUNCHXL_CryptoName
 *  @brief  Enum of Crypto names
 */
typedef enum CC2640R2_LAUNCHXL_CryptoName {
    CC2640R2_LAUNCHXL_CRYPTO0 = 0,

    CC2640R2_LAUNCHXL_CRYPTOCOUNT
} CC2640R2_LAUNCHXL_CryptoName;

/*!
 *  @def    CC2640R2_LAUNCHXL_GPIOName
 *  @brief  Enum of GPIO names
 */
typedef enum CC2640R2_LAUNCHXL_GPIOName {
    CC2640R2_LAUNCHXL_GPIO_S1 = 0,
    CC2640R2_LAUNCHXL_GPIO_S2,
    CC2640R2_LAUNCHXL_GPIO_LED_BLUE,
    CC2640R2_LAUNCHXL_GPIO_LED_RED,
    //CC2640R2_LAUNCHXL_GPIO_SPI_FLASH_CS,

    CC2640R2_LAUNCHXL_GPIOCOUNT
} CC2640R2_LAUNCHXL_GPIOName;

/*!
 *  @def    CC2640R2_LAUNCHXL_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC2640R2_LAUNCHXL_GPTimerName {
    CC2640R2_LAUNCHXL_GPTIMER0A = 0,
    CC2640R2_LAUNCHXL_GPTIMER0B,
    CC2640R2_LAUNCHXL_GPTIMER1A,
    CC2640R2_LAUNCHXL_GPTIMER1B,
    CC2640R2_LAUNCHXL_GPTIMER2A,
    CC2640R2_LAUNCHXL_GPTIMER2B,
    CC2640R2_LAUNCHXL_GPTIMER3A,
    CC2640R2_LAUNCHXL_GPTIMER3B,

    CC2640R2_LAUNCHXL_GPTIMERPARTSCOUNT
} CC2640R2_LAUNCHXL_GPTimerName;

/*!
 *  @def    CC2640R2_LAUNCHXL_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC2640R2_LAUNCHXL_GPTimers {
    CC2640R2_LAUNCHXL_GPTIMER0 = 0,
    CC2640R2_LAUNCHXL_GPTIMER1,
    CC2640R2_LAUNCHXL_GPTIMER2,
    CC2640R2_LAUNCHXL_GPTIMER3,

    CC2640R2_LAUNCHXL_GPTIMERCOUNT
} CC2640R2_LAUNCHXL_GPTimers;

/*!
 *  @def    CC2640R2_LAUNCHXL_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum CC2640R2_LAUNCHXL_I2CName {
    CC2640R2_LAUNCHXL_I2C0 = 0,

    CC2640R2_LAUNCHXL_I2CCOUNT
} CC2640R2_LAUNCHXL_I2CName;

/*!
 *  @def    CC2640R2_LAUNCHXL_NVSName
 *  @brief  Enum of NVS names
 */
typedef enum CC2640R2_LAUNCHXL_NVSName {
#ifndef Board_EXCLUDE_NVS_INTERNAL_FLASH
    CC2640R2_LAUNCHXL_NVSCC26XX0 = 0,
#endif
#ifndef Board_EXCLUDE_NVS_EXTERNAL_FLASH
    CC2640R2_LAUNCHXL_NVSSPI25X0,
#endif

    CC2640R2_LAUNCHXL_NVSCOUNT
} CC2640R2_LAUNCHXL_NVSName;

/*!
 *  @def    CC2640R2_LAUNCHXL_PWM
 *  @brief  Enum of PWM outputs
 */
typedef enum CC2640R2_LAUNCHXL_PWMName {
    CC2640R2_LAUNCHXL_PWM0 = 0,
    CC2640R2_LAUNCHXL_PWM1,
    CC2640R2_LAUNCHXL_PWM2,
    CC2640R2_LAUNCHXL_PWM3,
    CC2640R2_LAUNCHXL_PWM4,
    CC2640R2_LAUNCHXL_PWM5,
    CC2640R2_LAUNCHXL_PWM6,
    CC2640R2_LAUNCHXL_PWM7,

    CC2640R2_LAUNCHXL_PWMCOUNT
} CC2640R2_LAUNCHXL_PWMName;
/*!
 *  @def    CC2640R2_LAUNCHXL_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum CC2640R2_LAUNCHXL_SPIName {
    CC2640R2_LAUNCHXL_SPI0 = 0,
    CC2640R2_LAUNCHXL_SPI1,

    CC2640R2_LAUNCHXL_SPICOUNT
} CC2640R2_LAUNCHXL_SPIName;

/*!
 *  @def    CC2640R2_LAUNCHXL_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum CC2640R2_LAUNCHXL_UARTName {
    CC2640R2_LAUNCHXL_UART0 = 0,

    CC2640R2_LAUNCHXL_UARTCOUNT
} CC2640R2_LAUNCHXL_UARTName;

/*!
 *  @def    CC2640R2_LAUNCHXL_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2640R2_LAUNCHXL_UDMAName {
    CC2640R2_LAUNCHXL_UDMA0 = 0,

    CC2640R2_LAUNCHXL_UDMACOUNT
} CC2640R2_LAUNCHXL_UDMAName;

/*!
 *  @def    CC2640R2_LAUNCHXL_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum CC2640R2_LAUNCHXL_WatchdogName {
    CC2640R2_LAUNCHXL_WATCHDOG0 = 0,

    CC2640R2_LAUNCHXL_WATCHDOGCOUNT
} CC2640R2_LAUNCHXL_WatchdogName;

/*!
 *  @def    CC2650_LAUNCHXL_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum CC2640R2_LAUNCHXL_TRNGName {
    CC2640R2_LAUNCHXL_TRNG0 = 0,
    CC2640R2_LAUNCHXL_TRNGCOUNT
} CC2640R2_LAUNCHXL_TRNGName;

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_CC2640R2_MOUSEBOARD_H_ */
