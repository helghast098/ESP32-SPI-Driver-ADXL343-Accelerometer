#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "ADXL343.h"
#include "driver/spi_master.h"
#include "rom/ets_sys.h"

/*SPI COMMAND FIELD*/
#define CMD_WRITE_MODE  0x00 // Write to device
#define CMD_READ_MODE   0x02 // Read from device
#define CMD_MULTI_READ  0x01 // Auto increment Register
#define CMD_MULTI_WRITE 0x01 // Auto increment Register
#define CMD_SINGLE_READ 0x00 // No AUto increment Register

/*Device Register Fields*/
#define DEVID_R          0x00 // Device ID ( R )
#define THRESH_TAP_R     0x1D // Tap threshold ( R/W )
#define OFSX_R           0x1E // X-axis offset ( R/W )
#define OFSY_R           0x1F // Y-axis offset ( R/W )
#define OFSZ_R           0x20 // Z-axis offset ( R/W )
#define TAP_DUR_R        0x21 // Tap duration ( R/W )
#define TAP_LAT_R        0x22 // Tap latency ( R/W )
#define TAP_WIN_R        0x23 // Tap window ( R/W )
#define THRESH_ACT_R     0x24 // Activity threshold ( R/W )
#define THRESH_INACT_R   0x25 // Inactivity threshold ( R/W )
#define TIME_INACT_R     0x26 // Inactivity time ( R/W )pdMS_TO_TICKS( 10 )
#define ACT_INACT_CTL_R  0x27 // Axis enable control for activity and inactivity detection ( R )
#define THRESH_FF_R      0x28 // Free-fall threshold ( R/W )
#define TIME_FF_R        0x29 // Free-fall time ( R/W )
#define TAP_AXES_R       0x2A // Axis control for single tap/double tap ( R/W )
#define ACT_TAP_STATUS_R 0x2B // Source of single tap/double tap ( R )
#define BW_RATE_R        0x2C // Data rate and power mode control ( R/W )
#define POWER_CTL_R      0x2D // Power-saving features control ( R/W )
#define INT_ENABLE_R     0x2E // Interrupt enable control ( R/W )
#define INT_MAP_R        0x2F // Interrupt mapping control ( R/W )
#define INT_SOURCE_R     0x30 // Source of interrupts ( R )
#define DATA_FORMAT_R    0x31 // Data format control ( R/W )
#define DATAXO_R         0x32 // X-Axis Data 0 ( R )
#define DATAX1_R         0x33 // X-Axis Data 1 ( R )
#define DATAYO_R         0x34 // Y-Axis Data 0 ( R )
#define DATAY1_R         0x35 // Y-Axis Data 1 ( R )
#define DATAZO_R         0x36 // Z-Axis Data 0 ( R )
#define DATAZ1_R         0x37 // Z-Axis Data 1 ( R )
#define FIFO_CTL_R       0x38 // FIFO control ( R/W )
#define FIFO_STATUS_R    0x39 // FIFO status ( R )

/*mg/LSB*/
#define ADXL343_MG_FULL_RES 0.0039

/*GRAVITY*/
#define GRAVITY_STANDARD_EARTH 9.80665F

/*Duration Scale Factor*/
#define TAP_DUR_SCALE_FACTOR            625  // 625 us/LSB
#define TAP_DUR_MAX_TIME                TAP_DUR_SCALE_FACTOR * 0xFF // Max Tap Duration TIme
#define TAP_LATENCY_SCALE_FACTOR        1.25 // 1.25 ms/LSB
#define TAP_LATENCY_MAX_TIME            TAP_LATENCY_SCALE_FACTOR * 0xFF
#define SECOND_TAP_WINDOW_SCALE_FACTOR  1.25 // 1.25 ms/LSB
#define SECOND_TAP_WINDOW_MAX_TIME      TAP_LATENCY_SCALE_FACTOR * 0xFF
#define TAP_THRESHOLD_SCALE_FACTOR      0.0625 // 62.5 mg/LSB

struct ADXL343_Context_t {
	ADXL343_Config_t config;
	spi_device_handle_t devHandle;
};

/*Private Functions*/

/*Reads the lsb data of x y z: no conversion*/
esp_err_t Read_XYZ_Data_LSB(  const ADXL343_Handle_t handle, int16_t* x, int16_t* y, int16_t* z ) {

	esp_err_t err = ESP_OK;
	uint8_t data[6];

	spi_transaction_t trans = {
		.flags = 0,
		.cmd = CMD_READ_MODE | CMD_MULTI_READ,
		.addr = DATAXO_R,
		.length = 8 * 6,
		.rxlength = 8 * 6,
		.tx_buffer = NULL,
		.rx_buffer = data
	};

	err = spi_device_transmit( handle->devHandle, &trans );
	if ( err != ESP_OK ) return err;

	/*Converting result from registers to float val*/
	*x = ( ( int16_t ) ( (data[1] << 8) | data[0] ) );
	*y = ( ( int16_t ) ( (data[3] << 8) | data[2] ) );
	*z = ( ( int16_t ) ( (data[5] << 8) | data[4] ) );

	return ESP_OK;
}

/*returns output period of Accelerometer in ms*/
int Current_Output_Hz ( const ADXL343_Handle_t handle )  {
	if ( handle->config.outputRate <= ADXL343__3200__HZ && handle->config.outputRate >= ADXL343__100__HZ) return 10; // 10 ms
	else
	{
		switch ( handle->config.outputRate )
		{
			/*0.1Hz output rate*/
			case ADXL343__0_10__HZ:
				return ( 1 / 0.1 ) * 1000; // ms period

			/*0.2Hz output rate*/
			case ADXL343__0_20__HZ:
				return ( 1  / 0.2 ) * 1000;

			/*0.39Hz output rate*/
			case ADXL343__0_39__HZ:
				return ( 1 / 0.39 ) * 1000;

			/*0.78Hz output rate*/
			case ADXL343__0_78__HZ:
				return ( 1 / 0.78 ) * 1000;

			/*1.56Hz output rate*/
			case ADXL343__1_56__HZ:
				return ( 1 / 1.56 ) * 1000;

			/*3.13Hz output rate*/
			case ADXL343__3_13__HZ:
				return ( 1 / 3.13 ) * 1000;

			/*6.25Hz output rate*/
			case ADXL343__6_25__HZ:
				return ( 1 / 6.25 ) * 1000;

			/*12.5Hz output rate*/
			case ADXL343__12_5__HZ:
				return ( 1 / 12.5 ) * 1000;

			/*25Hz output rate*/
			case ADXL343__25__HZ:
				return ( 1 / 25.0 ) * 1000;

			/*50Hz output rate*/
			case ADXL343__50__HZ:
				return ( 1 / 50.0 ) * 1000;

			default:
				return 0;
		}

		return 0;
	}
}

/*Removes Offset when Device reset*/
esp_err_t Remove_Offset_Calibrate ( const ADXL343_Handle_t handle ) {
	/*Setting the offset registers of x y z*/
	esp_err_t err = ESP_OK;
	spi_transaction_t trans = {
		.flags = SPI_TRANS_USE_TXDATA,
		.cmd = CMD_WRITE_MODE | CMD_MULTI_WRITE,
		.addr = OFSX_R,
		.length = 24,
		.tx_data[0] = 0,
		.tx_data[1] = 0,
		.tx_data[2] = 0,
		.rx_buffer = NULL
	};

	err = spi_device_transmit( handle->devHandle, &trans );
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}

/*Write To Register*/
esp_err_t Write_To_Register( const ADXL343_Handle_t handle, uint8_t addr, uint8_t data ) {
	spi_transaction_t trans = {
		.flags = SPI_TRANS_USE_TXDATA,
		.cmd = CMD_WRITE_MODE | CMD_SINGLE_READ,
		.addr = addr,
		.rx_buffer = NULL,
		.tx_data[0] = data,
		.length = 8,
	};

	return spi_device_transmit( handle->devHandle, &trans );
}


/*Public Function*/
/*==================== Setting UP and Reading Accelerometer Values ====================*/
esp_err_t ADXL343_Init ( const ADXL343_Config_t* config, ADXL343_Handle_t* devHandle ) {
	esp_err_t err = ESP_OK;
// Delay for 10 ms
	if ( config == NULL || devHandle == NULL) return ESP_ERR_INVALID_ARG;

	/*Mallocing ADXLContext*/
	ADXL343_Context_t *context = ( ADXL343_Context_t * ) malloc( sizeof( ADXL343_Context_t ) );

	if ( context == NULL ) return ESP_ERR_NO_MEM;

	context->config = *config;

	/*Setting SPI Device Config*/
	spi_device_interface_config_t spiDevConfig = {
		.dummy_bits = 0,
		.command_bits = 2,
		.address_bits = 6,
		.mode = 3,
		.duty_cycle_pos = 128,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = context->config.clockSpeedHz > ADXL343_MAX_CLOCK_SPEED ? ADXL343_MAX_CLOCK_SPEED : context->config.clockSpeedHz,
		.input_delay_ns = 0,
		.spics_io_num = context->config.cs,
		.queue_size = 1,
		.pre_cb = NULL,
		.post_cb = NULL
	};
	err = spi_bus_add_device( context->config.hostID, &spiDevConfig, &( context->devHandle ) );

	/*If err occured when add device to SPI Bus*/
	if ( err != ESP_OK )
	{
		spi_bus_remove_device( context->devHandle );
		free( context );
		*devHandle = NULL;
		return err;
	}
	*devHandle = context;


	ADXL343_Stop_Measuring( context );
    ADXL343_Set_Output_Rate( context, context->config.outputRate );
	ADXL343_Enable_Interrupts( context, 0 ); // Disable Interrupts
	ADXL343_Set_Interrupt_Map( context, 0 ); // Set Interrupts to Pin 1

	Remove_Offset_Calibrate( context );
	return ESP_OK;
}

esp_err_t ADXL343_Start_Measuring ( const ADXL343_Handle_t handle ) {
	esp_err_t err = ESP_OK;

	/*First need to read POWER_CTL_R to save current setting*/
	spi_transaction_t trans = {
		.flags = SPI_TRANS_USE_RXDATA,
		.cmd = CMD_READ_MODE | CMD_SINGLE_READ,
		.addr = POWER_CTL_R,
		.tx_buffer = NULL,
		.length = 8,
		.rxlength = 8
	};

	err = spi_device_transmit( handle->devHandle, &trans );
	if ( err != ESP_OK ) return err;

	uint8_t currentRegVal = trans.rx_data[0];

	/*Setting measure bit and writing to reg*/
	currentRegVal |= 0x08;

	err = Write_To_Register( handle, POWER_CTL_R, currentRegVal );
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}

esp_err_t ADXL343_Stop_Measuring ( const ADXL343_Handle_t handle ) {
	esp_err_t err = ESP_OK;

	/*First need to read POWER_CTL_R to save current setting*/
	spi_transaction_t trans = {
		.flags = SPI_TRANS_USE_RXDATA,
		.cmd = CMD_READ_MODE | CMD_SINGLE_READ,
		.addr = POWER_CTL_R,
		.tx_buffer = NULL,
		.length = 8,
		.rxlength = 8
	};

	err = spi_device_transmit( handle->devHandle, &trans );
	if ( err != ESP_OK ) return err;

	uint8_t currentRegVal = trans.rx_data[0];

	/*Un Setting Measure Bit*/
	currentRegVal &= ( ~0x08 );

	err = Write_To_Register( handle, POWER_CTL_R, currentRegVal );
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}

esp_err_t ADXL343_Set_G_Range ( const ADXL343_Handle_t handle, uint8_t g ) {
	esp_err_t err = ESP_OK;
	/*First need to read POWER_CTL_R to save current setting*/
	uint8_t dataToSend = g > ADXL343_16G ? ADXL343_16G : g;
	dataToSend |= 0x08; // Full ress mode

	err = Write_To_Register( handle, DATA_FORMAT_R, dataToSend );
	if ( err != ESP_OK ) return err;

	return ESP_OK;

}

esp_err_t ADXL343_Set_Output_Rate(  const ADXL343_Handle_t handle, uint8_t rate ) {
	esp_err_t err = ESP_OK;

	err = Write_To_Register( handle, BW_RATE_R, rate );
	if ( err != ESP_OK ) return err;

	handle->config.outputRate = rate;

	return ESP_OK;
}

esp_err_t ADXL343_Read_XYZ_Data ( const ADXL343_Handle_t handle, float* x, float* y, float* z ) {
	if ( x == NULL || y == NULL || z == NULL ) return ESP_ERR_INVALID_ARG;

	esp_err_t err = ESP_OK;

	int16_t axisData[3];

	err = Read_XYZ_Data_LSB( handle, &axisData[0], &axisData[1], &axisData[2] );
	if (err != ESP_OK) return err;

	*x = ADXL343_MG_FULL_RES * axisData[0];
	*y = ADXL343_MG_FULL_RES * axisData[1];
	*z = ADXL343_MG_FULL_RES * axisData[2];

	return ESP_OK;
}

esp_err_t ADXL343_Calibrate ( const ADXL343_Handle_t handle ) {
	esp_err_t err = ESP_OK;

	int16_t x0g = 0;
	int16_t y0g = 0;
	int16_t z0g = 0;


	int waitInMS = Current_Output_Hz( handle );

	err = Remove_Offset_Calibrate( handle );

	for (int i = 0; i < 11; ++i)
	{

		int16_t x, y, z;
		Read_XYZ_Data_LSB( handle, &x, &y, &z );
		if (i == 0) continue;
		x0g += x;
		y0g += y;
		z0g += z;

		vTaskDelay( pdMS_TO_TICKS( waitInMS ) );
	}

	/*Getting Average of each axis*/
	x0g /= 10;
	y0g /= 10;
	z0g /= 10;

	z0g = ( 1 / ADXL343_MG_FULL_RES ) - z0g; // Subtracting with refrence to 1g

	x0g = -1 * round( x0g / 4.0 ); // Divide by 4 because offset regs 15.6mg/LSB
	y0g = -1 * round( y0g / 4.0 );
	z0g = round( z0g / 4.0 );

	/*Setting the offset registers of x y z*/
	spi_transaction_t trans = {
		.flags = SPI_TRANS_USE_TXDATA,
		.cmd = CMD_WRITE_MODE | CMD_MULTI_WRITE,
		.addr = OFSX_R,
		.length = 24,
		.tx_data[0] = (int8_t) x0g,
		.tx_data[1] = (int8_t) y0g,
		.tx_data[2] = (int8_t) z0g,
		.rx_buffer = NULL
	};

	err = spi_device_transmit( handle->devHandle, &trans );
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}


/*==================== FIRST AND SECOND TAP ====================*/
esp_err_t ADXL343_Set_Tap_Threshold ( const ADXL343_Handle_t handle, float gThreshold ) {
	esp_err_t err = ESP_OK;

	uint8_t gLSB = gThreshold / TAP_THRESHOLD_SCALE_FACTOR;

	err = Write_To_Register( handle, THRESH_TAP_R, gLSB );
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}

esp_err_t ADXL343_Set_Tap_Duration_Limit ( const ADXL343_Handle_t handle, uint32_t timeUS ) {
	esp_err_t err = ESP_OK;

	if ( timeUS > TAP_DUR_MAX_TIME ) return ESP_ERR_INVALID_ARG;

	uint8_t bitsInRegister = timeUS / TAP_DUR_SCALE_FACTOR;

	err = Write_To_Register( handle, TAP_DUR_R, bitsInRegister );
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}

esp_err_t ADXL343_Set_Tap_Latency ( const ADXL343_Handle_t handle, float timeMS ) {
	esp_err_t err = ESP_OK;

	if ( timeMS >  ( ( uint16_t ) TAP_LATENCY_MAX_TIME ) ) return ESP_ERR_INVALID_ARG;

	uint8_t bitsInRegister = timeMS / TAP_LATENCY_SCALE_FACTOR;

	err = Write_To_Register( handle, TAP_LAT_R, bitsInRegister );
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}

esp_err_t ADXL343_Set_Second_Tap_Window ( const ADXL343_Handle_t handle, float timeMS ) {
	esp_err_t err = ESP_OK;

	if ( timeMS >  ( ( uint16_t ) SECOND_TAP_WINDOW_MAX_TIME ) ) return ESP_ERR_INVALID_ARG;

	uint8_t bitsInRegister = timeMS / SECOND_TAP_WINDOW_SCALE_FACTOR;

	err = Write_To_Register( handle, TAP_WIN_R, bitsInRegister );
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}



/*==================== INTERRUPT SETUP ====================*/
esp_err_t ADXL343_Enable_Interrupts( const ADXL343_Handle_t handle, uint8_t interrupt ) {
	esp_err_t err = ESP_OK;

	err = Write_To_Register( handle, INT_ENABLE_R, interrupt );
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}

esp_err_t ADXL343_Set_Interrupt_Map( const ADXL343_Handle_t handle, uint8_t interruptMap ) {
	esp_err_t err = ESP_OK;

	err = Write_To_Register( handle, INT_MAP_R, interruptMap);
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}

esp_err_t ADXL343_Read_Interrupt_Source( const ADXL343_Handle_t handle, uint8_t* source ) {
	if (source == NULL) return ESP_ERR_INVALID_ARG;

	esp_err_t err = ESP_OK;

	spi_transaction_t trans = {
		.flags = SPI_TRANS_USE_RXDATA,
		.cmd = CMD_READ_MODE | CMD_SINGLE_READ,
		.addr = INT_SOURCE_R,
		.tx_buffer = NULL,
		.length = 8,
	};

	err = spi_device_transmit( handle->devHandle, &trans );
	if ( err != ESP_OK ) return err;

	*source = trans.rx_data[0];
	return ESP_OK;
}

esp_err_t ADXL343_Enable_Tap_Axes ( const ADXL343_Handle_t handle, uint8_t axes ) {
	esp_err_t err = ESP_OK;

	err = Write_To_Register( handle, TAP_AXES_R, axes );
	if ( err != ESP_OK ) return err;

	return ESP_OK;
}
