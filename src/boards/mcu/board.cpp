/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
	(C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

/******************************************************************************
 * @file    board.c
 * @author  Insight SiP
 * @version V2.0.0
 * @date    30-january-2019
 * @brief   Board (module) specific functions implementation.
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#include "board.h"

/** Semaphore used by SX126x IRQ handler to wake up LoRaWAN task */
SemaphoreHandle_t _lora_sem = NULL;

/** LoRa task handle */
TaskHandle_t _loraTaskHandle;
/** GPS reading task */
void _lora_task(void *pvParameters);

hw_config _hwConfig;

uint32_t lora_hardware_init(hw_config hwConfig)
{
	_hwConfig.CHIP_TYPE = hwConfig.CHIP_TYPE;					  // Chip type, SX1261 or SX1262
	_hwConfig.PIN_LORA_RESET = hwConfig.PIN_LORA_RESET;			  // LORA RESET
	_hwConfig.PIN_LORA_NSS = hwConfig.PIN_LORA_NSS;				  // LORA SPI CS
	_hwConfig.PIN_LORA_SCLK = hwConfig.PIN_LORA_SCLK;			  // LORA SPI CLK
	_hwConfig.PIN_LORA_MISO = hwConfig.PIN_LORA_MISO;			  // LORA SPI MISO
	_hwConfig.PIN_LORA_DIO_1 = hwConfig.PIN_LORA_DIO_1;			  // LORA DIO_1
	_hwConfig.PIN_LORA_BUSY = hwConfig.PIN_LORA_BUSY;			  // LORA SPI BUSY
	_hwConfig.PIN_LORA_MOSI = hwConfig.PIN_LORA_MOSI;			  // LORA SPI MOSI
	_hwConfig.RADIO_TXEN = hwConfig.RADIO_TXEN;					  // LORA ANTENNA TX ENABLE (e.g. eByte E22 module)
	_hwConfig.RADIO_RXEN = hwConfig.RADIO_RXEN;					  // LORA ANTENNA RX ENABLE (e.g. eByte E22 module)
	_hwConfig.USE_DIO2_ANT_SWITCH = hwConfig.USE_DIO2_ANT_SWITCH; // LORA DIO2 controls antenna
	_hwConfig.USE_DIO3_TCXO = hwConfig.USE_DIO3_TCXO;			  // LORA DIO3 controls oscillator voltage (e.g. eByte E22 module)
	_hwConfig.USE_DIO3_ANT_SWITCH = hwConfig.USE_DIO3_ANT_SWITCH; // LORA DIO3 controls antenna (e.g. Insight SIP ISP4520 module)
	_hwConfig.USE_LDO = hwConfig.USE_LDO;						  // LORA usage of LDO or DCDC power regulator (defaults to DCDC)
	_hwConfig.USE_RXEN_ANT_PWR = hwConfig.USE_RXEN_ANT_PWR;		  // RXEN used as power for antenna switch
	_hwConfig.TCXO_CTRL_VOLTAGE = hwConfig.TCXO_CTRL_VOLTAGE;

	SX126xIoInit();

	// After power on the sync word should be 2414. 4434 could be possible on a restart
	// If we got something else, something is wrong.
	uint16_t readSyncWord = 0;
	SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

	LOG_LIB("BRD", "SyncWord = %04X", readSyncWord);

	if ((readSyncWord == 0x2414) || (readSyncWord == 0x4434))
	{
		if (start_lora_task())
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	return 1;
}

uint32_t lora_hardware_re_init(hw_config hwConfig)
{
	_hwConfig.CHIP_TYPE = hwConfig.CHIP_TYPE;					  // Chip type, SX1261 or SX1262
	_hwConfig.PIN_LORA_RESET = hwConfig.PIN_LORA_RESET;			  // LORA RESET
	_hwConfig.PIN_LORA_NSS = hwConfig.PIN_LORA_NSS;				  // LORA SPI CS
	_hwConfig.PIN_LORA_SCLK = hwConfig.PIN_LORA_SCLK;			  // LORA SPI CLK
	_hwConfig.PIN_LORA_MISO = hwConfig.PIN_LORA_MISO;			  // LORA SPI MISO
	_hwConfig.PIN_LORA_DIO_1 = hwConfig.PIN_LORA_DIO_1;			  // LORA DIO_1
	_hwConfig.PIN_LORA_BUSY = hwConfig.PIN_LORA_BUSY;			  // LORA SPI BUSY
	_hwConfig.PIN_LORA_MOSI = hwConfig.PIN_LORA_MOSI;			  // LORA SPI MOSI
	_hwConfig.RADIO_TXEN = hwConfig.RADIO_TXEN;					  // LORA ANTENNA TX ENABLE (e.g. eByte E22 module)
	_hwConfig.RADIO_RXEN = hwConfig.RADIO_RXEN;					  // LORA ANTENNA RX ENABLE (e.g. eByte E22 module)
	_hwConfig.USE_DIO2_ANT_SWITCH = hwConfig.USE_DIO2_ANT_SWITCH; // LORA DIO2 controls antenna
	_hwConfig.USE_DIO3_TCXO = hwConfig.USE_DIO3_TCXO;			  // LORA DIO3 controls oscillator voltage (e.g. eByte E22 module)
	_hwConfig.USE_DIO3_ANT_SWITCH = hwConfig.USE_DIO3_ANT_SWITCH; // LORA DIO3 controls antenna (e.g. Insight SIP ISP4520 module)
	_hwConfig.USE_RXEN_ANT_PWR = hwConfig.USE_RXEN_ANT_PWR;		  // RXEN used as power for antenna switch

	SX126xIoReInit();

	// After power on the sync word should be 2414. 4434 could be possible on a restart
	// If we got something else, something is wrong.
	uint16_t readSyncWord = 0;
	SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

	LOG_LIB("BRD", "SyncWord = %04X", readSyncWord);

	if ((readSyncWord == 0x2414) || (readSyncWord == 0x4434))
	{
		if (start_lora_task())
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	return 1;
}

void _lora_task(void *pvParameters)
{
	LOG_LIB("BRD", "LoRa Task started");

	while (1)
	{
		if (xSemaphoreTake(_lora_sem, portMAX_DELAY) == pdTRUE)
		{
			// Handle Radio events
			Radio.BgIrqProcess();
		}
	}
}

bool start_lora_task(void)
{
	// Create the LoRaWan event semaphore
	_lora_sem = xSemaphoreCreateBinary();
	// Initialize semaphore
	xSemaphoreGive(_lora_sem);

	xSemaphoreTake(_lora_sem, 10);
	if (!xTaskCreate(_lora_task, "LORA", 4096, NULL, 1, &_loraTaskHandle))
	{
		return false;
	}
	return true;
}

void lora_hardware_uninit(void)
{
	vTaskSuspend(_loraTaskHandle);
	SX126xIoDeInit();
}

uint32_t BoardGetRandomSeed(void)
{
	return random(255);
}

void BoardGetUniqueId(uint8_t *id)
{
	uint64_t uniqueId = ESP.getEfuseMac();
	// Using ESP32 MAC (48 bytes only, so upper 2 bytes will be 0)
	id[7] = (uint8_t)(uniqueId >> 56);
	id[6] = (uint8_t)(uniqueId >> 48);
	id[5] = (uint8_t)(uniqueId >> 40);
	id[4] = (uint8_t)(uniqueId >> 32);
	id[3] = (uint8_t)(uniqueId >> 24);
	id[2] = (uint8_t)(uniqueId >> 16);
	id[1] = (uint8_t)(uniqueId >> 8);
	id[0] = (uint8_t)(uniqueId);
}

uint8_t BoardGetBatteryLevel(void)
{
	uint8_t batteryLevel = 0;

	// TO BE IMPLEMENTED

	return batteryLevel;
}

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR BoardDisableIrq(void)
{
	portENTER_CRITICAL(&mux);
}

void IRAM_ATTR BoardEnableIrq(void)
{
	portEXIT_CRITICAL(&mux);
}
