#include "touch.h"

extern I2C_HandleTypeDef hi2c3;

#define TP_I2C		&hi2c3
#define TP_ADDR		(0x41 << 1)

#define TP_ADC_FCT                0x01
#define TP_TP_FCT                 0x02
#define TP_IO_FCT                 0x04

/* Touch Panel Pins definition */
#define TOUCH_YD                   IO_Pin_1
#define TOUCH_XD                   IO_Pin_2
#define TOUCH_YU                   IO_Pin_3
#define TOUCH_XU                   IO_Pin_4
#define TOUCH_IO_ALL               (uint32_t)(IO_Pin_1 | IO_Pin_2 | IO_Pin_3 | IO_Pin_4)

/* IO Pins */
#define IO_Pin_0                   0x01
#define IO_Pin_1                   0x02
#define IO_Pin_2                   0x04
#define IO_Pin_3                   0x08
#define IO_Pin_4                   0x10
#define IO_Pin_5                   0x20
#define IO_Pin_6                   0x40
#define IO_Pin_7                   0x80
#define IO_Pin_ALL                 0xFF

static void touchReset(void)
{
	uint8_t sendBuf[2] = {0};
	sendBuf[0] = TP_REG_SYS_CTRL1;
	sendBuf[1] = 0x02;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);
	HAL_Delay(20);
	sendBuf[1] = 0x00;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);
	HAL_Delay(20);
}

static void fnctCmd(uint8_t fct, FunctionalState state)
{
	uint8_t tmp = 0;
	uint8_t sendBuf[2] = {0};
	sendBuf[0] = TP_REG_SYS_CTRL2;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 1, 100);
	HAL_I2C_Master_Receive(TP_I2C, TP_ADDR, sendBuf, 1, 100);
	tmp = sendBuf[0];

	if (state != DISABLE)
	{
		tmp &= ~(uint8_t)fct;
	}
	else
	{
		tmp |= (uint8_t)fct;
	}
	sendBuf[0] = TP_REG_SYS_CTRL2;
	sendBuf[1] = tmp;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);
}

static void ioAFConfig(uint8_t ioPin, FunctionalState state)
{
	uint8_t tmp = 0;
	uint8_t sendBuf[2] = {0};
	sendBuf[1] = TP_REG_GPIO_AF;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 1, 100);
	HAL_I2C_Master_Receive(TP_I2C, TP_ADDR, sendBuf, 1, 100);
	tmp = sendBuf[0];

	if (state != DISABLE)
	{
		tmp |= (uint8_t)ioPin;
	}
	else
	{
		tmp &= ~(uint8_t)ioPin;
	}

	sendBuf[0] = TP_REG_GPIO_AF;
	sendBuf[1] = tmp;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);
}

static void readX(uint16_t *x)
{
	int32_t _x;
	uint8_t readBuf[2] = {TP_REG_TP_DATA_X, 0};
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, readBuf, 1, 100);
	HAL_I2C_Master_Receive(TP_I2C, TP_ADDR, readBuf, 2, 100);

	_x = (readBuf[0] << 8) | readBuf[1];

	if (_x <= 3000)
	{
		_x = 3870 - _x;
	}
	else
	{
		_x = 3800 - _x;
	}

	*x = _x / 15;
	if (*x <= 0)
	{
		*x = 0;
	}
	else if (*x > 240)
	{
		*x = 239;
	}
}

static void readY(uint16_t *y)
{
	int32_t _y;
	uint8_t readBuf[2] = {TP_REG_TP_DATA_Y, 0};
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, readBuf, 1, 100);
	HAL_I2C_Master_Receive(TP_I2C, TP_ADDR, readBuf, 2, 100);

	_y = (readBuf[0] << 8) | readBuf[1];
	_y -= 360;
	*y = _y / 11;

	if (*y <= 0)
	{
		*y = 0;
	}
	else if (*y > 320)
	{
		*y = 319;
	}
}

static void readZ(uint16_t *z)
{
	uint8_t readBuf[2] = {TP_REG_TP_DATA_Z, 0};
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, readBuf, 1, 100);
	HAL_I2C_Master_Receive(TP_I2C, TP_ADDR, readBuf, 2, 100);

	*z = (readBuf[0] << 8) | readBuf[1];
	if (*z <= 0)
	{
		*z = 0;
	}
}

void TP_ReadID(uint16_t *id)
{
	uint8_t sendBuf[2] = {0};
	sendBuf[0] = TP_REG_CHP_ID;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 1, 100);
	HAL_I2C_Master_Receive(TP_I2C, TP_ADDR, sendBuf, 2, 100);
	*id = sendBuf[0] << 8;
	*id |= sendBuf[1];
}

void TP_Config(void)
{
	touchReset();
	fnctCmd(TP_ADC_FCT, ENABLE);
	fnctCmd(TP_TP_FCT, ENABLE);

	uint8_t sendBuf[2] = {TP_REG_ADC_CTRL1, 0x49};
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);
	HAL_Delay(20);

	sendBuf[0] = TP_REG_ADC_CTRL2;
	sendBuf[1] = 0x01;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);

	ioAFConfig((uint8_t)TOUCH_IO_ALL, DISABLE);

	sendBuf[0] = TP_REG_TP_CFG;
	sendBuf[1] = 0x9A;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);

	sendBuf[0] = TP_REG_FIFO_TH;
	sendBuf[1] = 0x01;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);

	sendBuf[0] = TP_REG_FIFO_STA;
	sendBuf[1] = 0x01;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);

	sendBuf[0] = TP_REG_FIFO_STA;
	sendBuf[1] = 0x00;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);

	sendBuf[0] = TP_REG_TP_FRACT_XYZ;
	sendBuf[1] = 0x01;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);

	sendBuf[0] = TP_REG_TP_I_DRIVE;
	sendBuf[1] = 0x01;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);

	sendBuf[0] = TP_REG_TP_CTRL;
	sendBuf[1] = 0x03;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);

	sendBuf[0] = TP_REG_INT_STA;
	sendBuf[1] = 0xFF;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, sendBuf, 2, 100);

}

void TP_GetState(tp_state_t *state)
{
	uint16_t xDiff, yDiff, x, y;
	static uint16_t _x = 0, _y = 0, _z = 0;

	uint8_t readBuf[2] = {TP_REG_TP_CTRL, 0};
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, readBuf, 1, 100);
	HAL_I2C_Master_Receive(TP_I2C, TP_ADDR, readBuf, 1, 100);

	state->touchDetected = (readBuf[0] & 0x80);
	if (state->touchDetected)
	{
		readX(&x);
		readY(&y);
		xDiff = x > _x? (x - _x): (_x - x);
		yDiff = y > _y? (y - _y): (_y - y);
		if (xDiff + yDiff > 5)
		{
			_x = x;
			_y = y;
		}

		readZ(&_z);
	}

	state->x = _x;
	state->y = _y;
	state->z = _z;

	readBuf[0] = TP_REG_FIFO_STA;
	readBuf[1] = 0x01;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, readBuf, 2, 100);
	readBuf[1] = 0x00;
	HAL_I2C_Master_Transmit(TP_I2C, TP_ADDR, readBuf, 2, 100);
}
