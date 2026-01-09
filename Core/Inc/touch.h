#include "stm32f4xx_hal.h"


/* Identification registers */
#define TP_REG_CHP_ID             0x00
#define TP_REG_ID_VER             0x02

/* General Control Registers */
#define TP_REG_SYS_CTRL1          0x03
#define TP_REG_SYS_CTRL2          0x04
#define TP_REG_SPI_CFG            0x08

/* Interrupt Control register */
#define TP_REG_INT_CTRL           0x09
#define TP_REG_INT_EN             0x0A
#define TP_REG_INT_STA            0x0B
#define TP_REG_GPIO_INT_EN        0x0C
#define TP_REG_GPIO_INT_STA       0x0D

/* GPIO Registers */
#define TP_REG_GPIO_SET_PIN       0x10
#define TP_REG_GPIO_CLR_PIN       0x11
#define TP_REG_GPIO_MP_STA        0x12
#define TP_REG_GPIO_DIR           0x13
#define TP_REG_GPIO_ED            0x14
#define TP_REG_GPIO_RE            0x15
#define TP_REG_GPIO_FE            0x16
#define TP_REG_GPIO_AF            0x17

/* ADC Registers */
#define TP_REG_ADC_INT_EN         0x0E
#define TP_REG_ADC_INT_STA        0x0F
#define TP_REG_ADC_CTRL1          0x20
#define TP_REG_ADC_CTRL2          0x21
#define TP_REG_ADC_CAPT           0x22
#define TP_REG_ADC_DATA_CH0       0x30
#define TP_REG_ADC_DATA_CH1       0x32
#define TP_REG_ADC_DATA_CH2       0x34
#define TP_REG_ADC_DATA_CH3       0x36
#define TP_REG_ADC_DATA_CH4       0x38
#define TP_REG_ADC_DATA_CH5       0x3A
#define TP_REG_ADC_DATA_CH6       0x3B
#define TP_REG_ADC_DATA_CH7       0x3C

/* TouchPanel Registers */
#define TP_REG_TP_CTRL            0x40
#define TP_REG_TP_CFG             0x41
#define TP_REG_WDM_TR_X           0x42
#define TP_REG_WDM_TR_Y           0x44
#define TP_REG_WDM_BL_X           0x46
#define TP_REG_WDM_BL_Y           0x48
#define TP_REG_FIFO_TH            0x4A
#define TP_REG_FIFO_STA           0x4B
#define TP_REG_FIFO_SIZE          0x4C
#define TP_REG_TP_DATA_X          0x4D
#define TP_REG_TP_DATA_Y          0x4F
#define TP_REG_TP_DATA_Z          0x51
#define TP_REG_TP_DATA_XYZ        0x52
#define TP_REG_TP_FRACT_XYZ       0x56
#define TP_REG_TP_DATA            0x57
#define TP_REG_TP_I_DRIVE         0x58
#define TP_REG_TP_SHIELD          0x59

typedef struct
{
  uint8_t 	touchDetected;
  uint16_t 	x;
  uint16_t 	y;
  uint16_t 	z;
}tp_state_t;

void TP_ReadID(uint16_t *id);
void TP_Config(void);
void TP_GetState(tp_state_t *state);
