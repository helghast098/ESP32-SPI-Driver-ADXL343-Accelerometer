# ESP32-SPI-Driver-ADXL343-Accelerometer
 
```bash
| README.md (This File)
├── ADXL343
│   │
│   ├── include # Header files
│   │   └── ADXL343.h # Function prototypes for Accelerometer.  Also contains custom data types for ADXL343
│   │  
│   └── src # Source files
│       └── ADXL343.c # Function definitions for Accelerometer
└── 
```
**Note:**<br>
The driver communicates with the ADXL343 with 4-wire SPI. Not all functionality of the ADXL343 is implemented: the measure and tap, data ready, and overrun interrupts are only setup.
Look at the ADXL343.h for more info on which functions are setup.

**How to Use:**<br>
Copy the directory ADXL343 into the components directory of the ESP-32 project.
