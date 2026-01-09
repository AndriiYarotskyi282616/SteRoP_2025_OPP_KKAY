#include "sdram.h"

FMC_SDRAM_CommandTypeDef command;

void SDRAM_init(SDRAM_HandleTypeDef *hsdram)
{
	__IO uint32_t tmpr = 0;

	command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
	command.AutoRefreshNumber = 1;
	command.ModeRegisterDefinition = 0;
	HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);
	HAL_Delay(10);
	command.CommandMode = FMC_SDRAM_CMD_PALL;
	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
	command.AutoRefreshNumber = 1;
	command.ModeRegisterDefinition = 0;
	HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);
	HAL_Delay(10);
	command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
	command.AutoRefreshNumber = 4;
	command.ModeRegisterDefinition = 0;
	HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);
	HAL_Delay(10);

	tmpr = (uint32_t) 0x01 | 0x00 | 0x30 | 0x00 | 0x200;
	command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
	command.AutoRefreshNumber = 1;
	command.ModeRegisterDefinition = tmpr;
	HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);
	HAL_Delay(10);

	HAL_SDRAM_ProgramRefreshRate(hsdram, 1386);
}
