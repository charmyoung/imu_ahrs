
/**********************************************************************
File    : uart.h
Purpose : 
**********************************************************************/
#ifndef __UART_H__
#define __UART_H__
/****************************** Includes *****************************/
/****************************** Defines *******************************/
/***************************** Prototypes *****************************/
void USART_Config(void);

//int fputc ( int ch );      // Primary UART for QUAT/ACCEL/GYRO data
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


#endif // __UART_H__


