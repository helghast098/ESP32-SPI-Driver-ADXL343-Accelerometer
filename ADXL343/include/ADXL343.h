#ifndef ADXL343_H
#define ADXL343_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

/*Accelerometer Max SPI clock speed*/
#define ADXL343_MAX_CLOCK_SPEED 5 * 1000 * 1000 // HZ

/*G Ranges: Used in Set_G_Range()*/
#define ADXL343_2G  0x00
#define ADXL343_4G  0x01
#define ADXL343_8G  0x10
#define ADXL343_16G 0x11

/*Output Data Rates for Measurments: Used in Set_Output_Rate()*/
#define ADXL343__0_10__HZ 0x00  // 0.10 Hz
#define ADXL343__0_20__HZ 0x01  // 0.20 Hz
#define ADXL343__0_39__HZ 0x02  // 0.39 Hz
#define ADXL343__0_78__HZ 0x03  // 0.78 Hz
#define ADXL343__1_56__HZ 0x04  // 1.56 Hz
#define ADXL343__3_13__HZ 0x05  // 3.13 Hz
#define ADXL343__6_25__HZ 0x06  // 6.25 Hz
#define ADXL343__12_5__HZ 0x07  // 12.5 Hz
#define ADXL343__25__HZ   0x08  // 25 Hz
#define ADXL343__50__HZ   0x09  // 50 Hz
#define ADXL343__100__HZ  0x0A  // 100 Hz
#define ADXL343__200__HZ  0x0B  // 200 Hz
#define ADXL343__400__HZ  0x0C  // 400 Hz
#define ADXL343__800__HZ  0x0D  // 800 Hz
#define ADXL343__1600__HZ 0x0E  // 1600 Hz
#define ADXL343__3200__HZ 0x0F  // 3200 Hz

/*Which Axis Should Tap Interrupt depend on can be ORed together: Used in Enable_Tap_Axes()*/
#define ADXL343_TAP_ENABLE_X_AXIS 0x06
#define ADXL343_TAP_ENABLE_Y_AXIS 0x02
#define ADXL343_TAP_ENABLE_Z_AXIS 0x01

/*========== Interrupts ==========*/

/*Enable Interrupts:  Used in Set_Interrupts()*/
#define ADXL343_INTR_DATA_READY 0x80  // Data Ready interrupt
#define ADXL343_INTR_SINGLE_TAP 0x40  // Single Tap Interrupt
#define ADXL343_INTR_DOUBLE_TAP 0x20  // Double Tap Interrupt
#define ADXL343_INTR_OVERRUN    0x01  // Overrun Interrupt

/* Interrupt Map to pin 1 or 2:  Used in Set_Interrupt_Map().*/
/*Data Ready*/
#define ADXL343_PIN_1_DATA_READY 0x00 // Alert on Pin 1
#define ADXL343_PIN_2_DATA_READY 0x80 // Alert on Pin 2

/*Single Tap*/
#define ADXL343_PIN_1_SINGLE_TAP 0x00 // Alert on Pin 1
#define ADXL343_PIN_2_SINGLE_TAP 0x40 // Alert on Pin 2

/*Dobule Tap*/
#define ADXL343_PIN_1_DOUBLE_TAP 0x00 // Alert on Pin 1
#define ADXL343_PIN_2_DOUBLE_TAP 0x20 // Alert on Pin 2

/*Watermark*/
#define ADXL343_PIN_1_WATERMARK 0x00 // Alert on Pin 1
#define ADXL343_PIN_2_WATERMARK 0x02 // Alert on Pin 2


/*!!!!!!!!!!NOT IMPLEMENTED!!!!!!!!!!!*/
#define ADXL343_INTR_WATERMARK   0x00
#define ADXL343_INTR_ACTIVITY    0x00
#define ADXL343_INTR_INACTIVITY  0x00
#define ADXL343_INTR_FREEFALL    0x00

#define ADXL343_OVERRUN_PIN_1    0x00
#define ADXL343_OVERRUN_PIN_2    0x00
#define ADXL343_ACTIVITY_PIN_1   0x00
#define ADXL343_ACTIVITY_PIN_2   0x00
#define ADXL343_INACTIVITY_PIN_1 0x00
#define ADXL343_INACTIVITY_PIN_2 0x00
#define ADXL343_FREEFALL_PIN_1   0x00
#define ADXL343_FREEFALL_PIN_2   0x00
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/*Configure struct used to initiliaze Accelerometer*/
typedef struct {
	spi_host_device_t hostID;
	int clockSpeedHz;
	gpio_num_t cs;
	uint8_t outputRate; // Use one of the Defined output rates
} ADXL343_Config_t;

typedef struct ADXL343_Context_t ADXL343_Context_t;

/*Handle for accelerometer*/
typedef ADXL343_Context_t* ADXL343_Handle_t;


/**
* @brief Initialize ADXL343 Accelerometer
*
* @param config: pointer to ADXL343_Config_t struct
* @param devHandle: pointer to device handle to use for all functions
* @return
*		- ESP_OK: On success
*		- ESP_ERR_NO_MEM
*       - ESP_ERR_TIMEOUT
*
*/
esp_err_t ADXL343_Init ( const ADXL343_Config_t* config, ADXL343_Handle_t* devHandle );

/**
* @brief Start measuring acceleration
*
* @param handle: pointer to ADXL343 Handle
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Start_Measuring ( const ADXL343_Handle_t handle );

/**
* @brief Stop measuring acceleration
*
* @param handle: pointer to ADXL343 Handle
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Stop_Measuring ( const ADXL343_Handle_t handle );

/**
* @brief Change range of measurable G
*
* @param handle: pointer to ADXL343 Handle
* @param g: One of the G Ranges Defines
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Set_G_Range ( const ADXL343_Handle_t handle, uint8_t g );

/**
* @brief Sets the output rate of measurements
*
* @param handle: pointer to ADXL343 Handle
* @param rate: Must be one of the DEFINED rates
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Set_Output_Rate ( const ADXL343_Handle_t handle, uint8_t rate );

/**
* @brief Read XYZ Data. Unit is in Gs
*
* @param handle: pointer to ADXL343 Handle
* @param x: Measured x axis value is returned here to user
* @param y: Measured y axis value is returned here to user
* @param z: Measured z axis value is returned here to user
* @return
*		- ESP_OK: On success
*		- ESP_ERR_INVALID_ARG: If either float address is NULL
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Read_XYZ_Data ( const ADXL343_Handle_t handle, float* x, float* y, float* z );

/**
* @brief Calibrates Accelerometer to zero is axis. Make
* @note: Make sure the Z axis is pointing up
*
* @param handle: pointer to ADXL343 Handle
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/

/*==================== FIRST AND SECOND TAP ====================*/
esp_err_t ADXL343_Calibrate ( const ADXL343_Handle_t handle );

/**
* @brief Set Tap Threshold in which tap will register when over
* @note Make sure to call ADXL343_Start_Measuring() before calling this function
*
* @param handle: pointer to ADXL343 Handle
* @param gThreshold: float value must range from 0g-16g
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Set_Tap_Threshold ( const ADXL343_Handle_t handle, float gThreshold);

/**
* @brief Set Tap Duration: Length in which tap must stay below to register
* @note Has a scale factor of 625 us/LSB so use incremnts of 625 us
*
* @param handle: pointer to ADXL343 Handle
* @param timeUS: time in microseconds: range from 0us-159,375us
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Set_Tap_Duration_Limit ( const ADXL343_Handle_t handle, uint32_t timeUS );

/**
* @brief Set Tap Latency after first tap: Used for second tap register
* @note Since scale factor is 1.25 ms/LSB use increments of 1.25 ms
*		A value of 0 turns off second tap.
*
* @param handle: pointer to ADXL343 Handle
* @param timeMS: float value must range from 0ms-318.75ms
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Set_Tap_Latency ( const ADXL343_Handle_t handle, float timeMS );

/**
* @brief Set Second Tap Window: Time range in which second tap must be detected
* @note Since scale factor is 1.25 ms/LSB use increments of 1.25 ms
*		A value of 0 turns off second tap.
*
* @param handle: pointer to ADXL343 Handle
* @param timeMS: float value must range from 0ms-318.75ms
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Set_Second_Tap_Window ( const ADXL343_Handle_t handle, float timeMS );

/**
* @brief Sets which axes are used to trigger the Tap Event
*
*
* @param handle: pointer to ADXL343 Handle
* @param axes: The DXL343_TAP_ENABLE_..... Defines must be used. Can also be ORed together
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Enable_Tap_Axes ( const ADXL343_Handle_t handle, uint8_t axes );

/*==================== INTERRUPT SETUP ====================*/

/**
* @brief Enables the Accelerometer interrupts such as Tap, Double Tap, Data Ready, etc.
*
* @param handle: pointer to ADXL343 Handle
* @param interrupt: Takes in ORed Value of ADXL343_INTR_...
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Enable_Interrupts( const ADXL343_Handle_t handle, uint8_t interrupt );

/**
* @brief Determines which pin (1 or 2) should go high when interrupt triggered
*
* @param handle: pointer to ADXL343 Handle
* @param interrupt: Takes in ORed Value of ADXL343_PIN_1... or ADXL343_PIN_2...
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Set_Interrupt_Map( const ADXL343_Handle_t handle, uint8_t interruptMap );

/**
* @brief Determines which pin (1 or 2) should go high when interrupt triggered
* @Note To see which interrupt was triggered, source and one of ADXL343_INTR_...
*		must be AND together
* @param handle: pointer to ADXL343 Handle
* @param source: Returned value that contain which interrupt triggered
* @return
*		- ESP_OK: On success
*		- Other Error Depending on spi_device_transmit
*
*/
esp_err_t ADXL343_Read_Interrupt_Source( const ADXL343_Handle_t handle, uint8_t *source );
#endif
