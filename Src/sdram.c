#include "sdram.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal_sdram.h"
#include "stm32f4xx_ll_fmc.h"


/* SDRAM handler declaration */
SDRAM_HandleTypeDef hsdram;
FMC_SDRAM_TimingTypeDef SDRAM_Timing;
FMC_SDRAM_CommandTypeDef command;

void SDRAM_GPIO_Init(SDRAM_HandleTypeDef *hsdram);

static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
	__IO uint32_t tmpmrd =0;
	/* Step 3:  Configure a clock configuration enable command */
	Command->CommandMode 			 = FMC_SDRAM_CMD_CLK_ENABLE;
	Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber 	 = 1;
	Command->ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

	/* Step 4: Insert 100 ms delay */
	HAL_Delay(100);

	/* Step 5: Configure a PALL (precharge all) command */ 
	Command->CommandMode 			 = FMC_SDRAM_CMD_PALL;
	Command->CommandTarget 	     = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber 	 = 1;
	Command->ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);  

	/* Step 6 : Configure a Auto-Refresh command */ 
	Command->CommandMode 			 = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber 	 = 4;
	Command->ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

	/* Step 7: Program the external memory mode register */
	tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2          |
		SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
		SDRAM_MODEREG_CAS_LATENCY_3           |
		SDRAM_MODEREG_OPERATING_MODE_STANDARD |
		SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

	Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
	Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber 	 = 1;
	Command->ModeRegisterDefinition = tmpmrd;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

	/* Step 8: Set the refresh rate counter */
	/* (15.62 us x Freq) - 20 */
	/* Set the device refresh counter */
	HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT); 
}


bool Init_SDRAM_Device()
{

	SDRAM_GPIO_Init(&hsdram);

	/* SDRAM device configuration */ 
	hsdram.Instance = FMC_SDRAM_DEVICE;

	/* Timing configuration for 90 MHz of SD clock frequency (180MHz/2) */
	/* TMRD: 2 Clock cycles */
	SDRAM_Timing.LoadToActiveDelay    = 2;
	/* TXSR: min=70ns (6x11.90ns) */
	SDRAM_Timing.ExitSelfRefreshDelay = 7;
	/* TRAS: min=42ns (4x11.90ns) max=120k (ns) */
	SDRAM_Timing.SelfRefreshTime      = 4;
	/* TRC:  min=63 (6x11.90ns) */        
	SDRAM_Timing.RowCycleDelay        = 7;
	/* TWR:  2 Clock cycles */
	SDRAM_Timing.WriteRecoveryTime    = 2;
	/* TRP:  15ns => 2x11.90ns */
	SDRAM_Timing.RPDelay              = 2;
	/* TRCD: 15ns => 2x11.90ns */
	SDRAM_Timing.RCDDelay             = 2;

	hsdram.Init.SDBank             = FMC_SDRAM_BANK2;
	hsdram.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8;
	hsdram.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram.Init.MemoryDataWidth    = SDRAM_MEMORY_WIDTH;
	hsdram.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram.Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_3;
	hsdram.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram.Init.SDClockPeriod      = SDCLOCK_PERIOD;
	hsdram.Init.ReadBurst          = FMC_SDRAM_RBURST_DISABLE;
	hsdram.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_1;

	/* Initialize the SDRAM controller */
	if(HAL_SDRAM_Init(&hsdram, &SDRAM_Timing) != HAL_OK)
	{
		/* Initialization Error */
		return false;
	}

	/* Program the SDRAM external device */
	SDRAM_Initialization_Sequence(&hsdram, &command);

	return true;
}

void SDRAM_GPIO_Init(SDRAM_HandleTypeDef *hsdram)
{
  GPIO_InitTypeDef  GPIO_Init_Structure;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clocks */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  /* Enable FMC clock */
  __HAL_RCC_FMC_CLK_ENABLE();
                            
  /*##-2- Configure peripheral GPIO ##########################################*/  
/*-- GPIOs Configuration -----------------------------------------------------*/
/*
 +-------------------+--------------------+--------------------+--------------------+
 +                       SDRAM pins assignment                                      +
 +-------------------+--------------------+--------------------+--------------------+
 | PD0  <-> FMC_D2   | PE0  <-> FMC_NBL0  | PF0  <-> FMC_A0    | PG0  <-> FMC_A10   |
 | PD1  <-> FMC_D3   | PE1  <-> FMC_NBL1  | PF1  <-> FMC_A1    | PG1  <-> FMC_A11   |
 | PD8  <-> FMC_D13  | PE7  <-> FMC_D4    | PF2  <-> FMC_A2    | PG8  <-> FMC_SDCLK |
 | PD9  <-> FMC_D14  | PE8  <-> FMC_D5    | PF3  <-> FMC_A3    | PG15 <-> FMC_NCAS  |
 | PD10 <-> FMC_D15  | PE9  <-> FMC_D6    | PF4  <-> FMC_A4    |--------------------+ 
 | PD14 <-> FMC_D0   | PE10 <-> FMC_D7    | PF5  <-> FMC_A5    |   
 | PD15 <-> FMC_D1   | PE11 <-> FMC_D8    | PF11 <-> FMC_NRAS  | 
 +-------------------| PE12 <-> FMC_D9    | PF12 <-> FMC_A6    | 
                     | PE13 <-> FMC_D10   | PF13 <-> FMC_A7    |    
                     | PE14 <-> FMC_D11   | PF14 <-> FMC_A8    |
                     | PE15 <-> FMC_D12   | PF15 <-> FMC_A9    |
 +-------------------+--------------------+--------------------+
 | PB5 <-> FMC_SDCKE1| 
 | PB6 <-> FMC_SDNE1 | 
 | PC0 <-> FMC_SDNWE |
 +-------------------+  
  
*/
  
  /* Common GPIO configuration */
  GPIO_Init_Structure.Mode  = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Speed = GPIO_SPEED_FAST;
  GPIO_Init_Structure.Pull  = GPIO_NOPULL;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FMC;

  /* GPIOB configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOB, &GPIO_Init_Structure);  

  /* GPIOC configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_0;      
  HAL_GPIO_Init(GPIOC, &GPIO_Init_Structure);  
  
  /* GPIOD configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1  | GPIO_PIN_8 |
                                GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 |
                                GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* GPIOE configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_7 |
                                GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 |
                                GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                                GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);

  /* GPIOF configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_0  | GPIO_PIN_1 | GPIO_PIN_2 | 
                                GPIO_PIN_3  | GPIO_PIN_4 | GPIO_PIN_5 |
                                GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                                GPIO_PIN_14 | GPIO_PIN_15;      
  HAL_GPIO_Init(GPIOF, &GPIO_Init_Structure);

  /* GPIOG configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 |
                                GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);

}
