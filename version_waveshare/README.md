#  Migration Version Beta 1.0
## HARDWARE
#### MCU: STM32F4 Waveshare Board
#### Inertial Sensor: MPU9250 (9 axis)
## IDE
#### MDK5.20 & KEIL
## Library
#### eMPL & DMP
- More information on invensense motion driver manual
## Migration & Modification
####  **Origin Version** is based on [STM32 Forum](http://www.stmcu.org/module/forum/forum.php?mod=viewthread&tid=602861&extra=&authorid=1670729&page=1), Thanks to @watershade.
#### **Connection:**
    MPU9250  - STM32F4
    VCC - VCC(3.3V)
    GND - GND
    SCL - PB10
    SDA - PB11
    AD0 - GND
    INT output - PA1
    NCS - VCC(3.3V)

	STM32F4 - CH340
	USART2 Tx(PA2)- RX
	USART2 Rx(PA3)- TX
	VCC - VCC
	GND - GND
#### **File Structure**
- STM32F4xx_StdPeriph_Driver
  - inc{}
  - src{}
- core
  - driver
    - eMPL{}
    - include{}
    - stm32L{}
  - eMPL-hal{}
  - mllite{}
   - mpl
- DeviceSupport
  - inc{}
  - src{}
- Peripheral
  - inc{}
  - src{}
- User
  - inc{}
  - src{}    
- MDK-ARM
- Binary
- CMSIS
#### **Preprocessor Symbols**
	USE_STDPERIPH_DRIVER, USE_DMP, MPL_LOG_NDEBUG=1, EMPL, MPU9250, STM32F40_41xxx, EMPL_TARGET_STM32F4
#### **Float Operation**
- KEIL: Target>Roating Point Hardware>Usr Single Precision
#### **Stack**
- KEIL: Tagart>IROM1/IRAM1(by default),
- Modify .s file and relevent .h file in mpl, 
- Replace .a for .lib in Device Support
#### **Debug**
- STlink
### **APPLICATION IN PYTHON**
- Installing Python 2.7 (32-bits version) or above, pyserial, and pygame
  - Python
  - Pyserial
  - Pygame


