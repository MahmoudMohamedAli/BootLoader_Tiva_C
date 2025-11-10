//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2012-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "driverlib/flash.h"

#include "crc.h"
#include "packet.h"

#define LEDBASE GPIO_PORTF_BASE
#define LEDRED GPIO_PIN_1
#define LEDBLUE GPIO_PIN_2
#define LEDGREEN GPIO_PIN_3

#define SWBASE GPIO_PORTF_BASE
#define SW1 GPIO_PIN_4
#define SW2 GPIO_PIN_0

#define APP_START_ADDRESS ((uint32_t)0x00006000U)

#define MAX_STRING_LEN 260
#define TERMINATOR 0xA5 // Use Line Feed as the end-of-message marker



strPacket_t rxBuffer;
strPacket_t txBuffer;

uint32_t flsDrvBuffer[64];
uint8_t flsDrvBufferInd = 0;

uint32_t payloadSize;

crc_t crc;


char g_RxBuffer[MAX_STRING_LEN]; // Buffer to hold the assembled string
int g_BufferIndex = 0;           // Current index in the buffer
volatile bool g_NewStringReady = false; // Flag for the main loop

void toggleLed();
void Process_UART_Data();


typedef enum enuState{
    BL_STATE_IDLE,
    BL_STATE_ERASE_STARTED,
    BL_STATE_WRITE_STARTED,
    BL_STATE_WRITE_FINISHED,
} enuState_t;

static enuState_t BL_State = BL_STATE_IDLE;

static uint32_t flashIndex = APP_START_ADDRESS; // first app address
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

// Function to clear the receive buffer
void UART_FlushRx(uint32_t ui32Base) {
    // Read and discard all characters until the FIFO is empty
    while (UARTCharsAvail(ui32Base)) {
        UARTCharGetNonBlocking(ui32Base);
    }
}

char UART_ReceiveChar_Polling(void);


// TI ARMCL: Use __attribute__((naked)) and pass arguments in R0, R1
void __attribute__((naked)) jump_to_app1(uint32_t new_sp, uint32_t new_pc)
{
    asm(" MOV SP, R0 ");
    asm(" BX R1 ");
    // No return; processor jumps out!
}

void jump_to_app(void) {
    uint32_t app_vtor = 0x00006000;
    uint32_t *vtor = (uint32_t *)0xE000ED08;
    *vtor = app_vtor;
    uint32_t new_sp = ((uint32_t *)app_vtor)[0];
    uint32_t new_pc = ((uint32_t *)app_vtor)[1];
     jump_to_app1(new_sp,new_pc);
   // asm("mov sp, %0" : : "r"(new_sp));
    //asm("bx %0" : : "r"(new_pc));
}
//*****************************************************************************
//
// Print "Hello World!" to the UART on the evaluation board.
//
//*****************************************************************************
int
main(void)
{
    //volatile uint32_t ui32Loop;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    MAP_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        //wait till the portf is Ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));   

           // Unlock PF0
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;

    // Set push buttons pins as input (PF0,PF4)
    GPIOPinTypeGPIOInput(SWBASE, SW1 | SW2);
    // Enable pullup resistors on PF0,PF4
    GPIOPadConfigSet(SWBASE,SW1 | SW2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

    // Erased application check
    uint64_t *pAppArea = (uint64_t *) 0x00006000U;
   
    uint8_t erasedAppFlag = *pAppArea == 0xFFFFFFFFFFFFFFFFU ? 1 : 0;

      // If pushbuttons were not pressed, jump to application.
    if (GPIOPinRead(SWBASE,/*SW2 | */SW1)
            && erasedAppFlag == 0x00) {
        // Reinit used peripherals
        SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOF);
        // __asm__ volatile (
        //     // Update vector table offset to application vector table
        //     "ldr     r0, = 0x00006000\n"
        //     "ldr     r1, = 0xe000ed08\n"
        //     "str     r0, [r1]\n"

        //     // Update stack pointer from application vector table. First entry of vector table is SP
        //     "ldr     r1, [r0]\n"
        //     "mov     sp, r1\n"

        //     // Load application reset handler and jump to the user code
        //     "ldr     r0, [r0, #4]\n"
        //     "bx      r0\n"
        //     );
        jump_to_app();
//         void jump_to_app(void) {
//     uint32_t app_vtor = 0x00006000;
//     uint32_t *vtor = (uint32_t *)0xE000ED08;
//     *vtor = app_vtor;
//     uint32_t new_sp = ((uint32_t *)app_vtor)[0];
//     uint32_t new_pc = ((uint32_t *)app_vtor)[1];

//     asm("mov sp, %0" : : "r"(new_sp));
//     asm("bx %0" : : "r"(new_pc));
// }
    }
          // Enable GPIOA clock for UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    // Enable UART0 clock
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

    // Enable UART on PA0/PA1
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // UART Init
    UARTConfigSetExpClk(CFG_UART_BASE, 16000000, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));

GPIOPinTypeGPIOOutput(LEDBASE, LEDRED | LEDBLUE | LEDGREEN);
//     Initialize the UART.
        GPIOPinWrite(LEDBASE, LEDBLUE, LEDBLUE);
  // ConfigureUART();

   // char byte = 0xDF;
   // UARTWriteByte(CFG_UART_BASE, byte);
    //UARTprintf("BootLoader State!\n");
    // Set LEDs pins type as output
   // GPIOPinTypeGPIOOutput(LEDBASE, LEDRED | LEDBLUE | LEDGREEN);

    // Initialize CRC lib
    crc = crc_init();


  //  UARTprintf("BootLoader State!\n");
    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    //
  //  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
//     Initialize the UART.
    
   ConfigureUART();
  //  UARTprintf("Boot\n");

    //
    // Hello!
    //
   // UARTprintf("Hello, world!\n");
  // UART_FlushRx(UART0_BASE);
//UARTprintf("BootLoader State!\n");
    //
    // We are finished.  Hang around doing nothing.
    //
    while(1)
    {
       // UARTprintf("BootLoader\n");

        //UARTprintf("ACK\n");
        //char byte  = UART_ReceiveChar_Polling();
        //UARTprintf(&byte );
       // UARTprintf("\n" );

    //     Process_UART_Data();
    //    // printf("Data Received: %s", g_RxBuffer);
    //    if (g_NewStringReady) {
    //         // The complete string is in g_RxBuffer
    //         // Example: Print the string back
    //         UARTprintf("Received: %s\n", g_RxBuffer);
    //        //  UARTprintf("Send: Ack for %s\n", g_RxBuffer);
    //         g_NewStringReady = false;
    //    }

   rcvPacket(&rxBuffer);



  //  UARTprintf("rxBuffer:%d\n", rxBuffer.packetOpcode);
  //  UARTprintf("Packet valid:%d\n", rxBuffer.packetValid);
        if (rxBuffer.packetValid) {
            //UARTprintf("Data Valid\n");
            // Erase request
            if (rxBuffer.packetOpcode == 1 && BL_State == BL_STATE_IDLE) {

                BL_State = BL_STATE_ERASE_STARTED;
                uint32_t appArea;
                for(appArea = APP_START_ADDRESS; appArea < 0x00040000; appArea+=0x400) {
                    GPIOPinWrite(LEDBASE, LEDRED, LEDRED);
                    if (FlashErase(appArea) != 0) {
                        // Erasing flash failed. Send error and break.
                        txBuffer.packetOpcode = 0xA1;
                        txBuffer.dataLen = 1;
                        // erase failed
                        txBuffer.packetData[0] = 0xF0;
                        sendPacket(&txBuffer);
                        break;

                    } else {
                        GPIOPinWrite(LEDBASE, LEDRED, 0);
                    }

               }

                // Send erase end indication
                txBuffer.packetOpcode = 0xA1;
                txBuffer.dataLen = 1;
                // erase finished
                txBuffer.packetData[0] = 1;

                sendPacket(&txBuffer);

                BL_State = BL_STATE_IDLE;

            // Flash request
            } else if (rxBuffer.packetOpcode == 0x02 && BL_State == BL_STATE_IDLE) {
                payloadSize = bytesToU32(rxBuffer.packetData, 0);

                txBuffer.packetOpcode = 0xA2;
                txBuffer.dataLen = 0x01;

                if (payloadSize <= (0x00040000 - APP_START_ADDRESS)) {
                    flashIndex = APP_START_ADDRESS;
                    BL_State = BL_STATE_WRITE_STARTED;
                    crc = crc_init();
                    txBuffer.packetData[0] = 0x01; // flashing accepted
                } else {
                    txBuffer.packetData[0] = 0x00; // flashing refused
                }

                sendPacket(&txBuffer);

            // Flash data
            } else if (rxBuffer.packetOpcode == 0x03 && BL_State == BL_STATE_WRITE_STARTED) {
                if (((rxBuffer.dataLen % 4) == 0)
                        && ((flashIndex + rxBuffer.dataLen) <= 0x00040000))  {

                    // Concatenate every 4 bytes into an UInt32 because flash driver expects an array of UInt32s
                    // TM4C123 is little-endian.
                    flsDrvBufferInd = 0;
                    int i;
                    for (i = 0; i < rxBuffer.dataLen;) {
                        flsDrvBuffer[flsDrvBufferInd] = rxBuffer.packetData[i++];
                        flsDrvBuffer[flsDrvBufferInd] |= rxBuffer.packetData[i++] << 8;
                        flsDrvBuffer[flsDrvBufferInd] |= rxBuffer.packetData[i++] << 16;
                        flsDrvBuffer[flsDrvBufferInd] |= rxBuffer.packetData[i++] << 24;
                        flsDrvBufferInd++;
                    }

                    int32_t writeRes = FlashProgram(flsDrvBuffer,flashIndex,rxBuffer.dataLen);

                    // prepare response
                    txBuffer.packetOpcode = 0xA3;
                    txBuffer.dataLen = 0x01;

                    if (writeRes == 0) {
                        // success

                        flashIndex += rxBuffer.dataLen;

                        // update CRC
                        crc = crc_update(crc, rxBuffer.packetData, rxBuffer.dataLen);

                        if ((flashIndex - APP_START_ADDRESS) < payloadSize) {
                            txBuffer.packetData[0] = 0x01; // send next chunk of data
                        } else {
                            txBuffer.packetData[0] = 0x02; // application is completely transferred.
                            BL_State = BL_STATE_WRITE_FINISHED;
                        }

                    } else {
                        txBuffer.packetData[0] = 0x00; // failed
                    }
                    sendPacket(&txBuffer);

                } else {
                    txBuffer.packetOpcode = 0xA3;
                    txBuffer.dataLen = 0x01;
                    txBuffer.packetData[0] = 0xFF; // overflow
                    sendPacket(&txBuffer);
                }

            // End flash
            } else if (rxBuffer.packetOpcode == 0x04 && BL_State == BL_STATE_WRITE_FINISHED) {
                crc = crc_finalize(crc);

                txBuffer.packetOpcode = 0xA4;
                txBuffer.dataLen = 1;

                uint32_t rcvdCrc = bytesToU32(rxBuffer.packetData,0);

                if (rcvdCrc != crc) {
                    // mismatch
                    txBuffer.packetData[0] = 0;
                } else {
                    // correct
                    txBuffer.packetData[0] = 1;
                }

                sendPacket(&txBuffer);

                BL_State = BL_STATE_IDLE;
            // Restart request
            } else if (rxBuffer.packetOpcode == 0x05) {
                SysCtlReset();
            }
            // Get PC (Just for testing code execution from RAM)
            // } else if (rxBuffer.packetOpcode == 0x06) {
            //     // send PC
            //     uint32_t pc = _get_PC();
            //     txBuffer.packetOpcode = 0xA6;
            //     txBuffer.dataLen = 4;
            //     txBuffer.packetData[0] = (pc & 0xFF000000) >> 24;
            //     txBuffer.packetData[1] = (pc & 0x00FF0000) >> 16;
            //     txBuffer.packetData[2] = (pc & 0x0000FF00) >> 8;
            //     txBuffer.packetData[3] = pc & 0x000000FF;
            //     sendPacket(&txBuffer);
            // }
        }
    }
}


void Process_UART_Data() 
{
    char received_byte;

    // Check the Ring Buffer (or use the TivaWare function)
    while (!UARTCharsAvail(UART0_BASE)) {
        //   toggleLed();
    }
    if (UARTCharsAvail(UART0_BASE)) {
        received_byte = UARTCharGet(UART0_BASE);
        
        // 1. Check for the Terminator
        if (received_byte == TERMINATOR) {
            // Null-terminate the string to make it a proper C-string
            g_RxBuffer[g_BufferIndex] = '\0'; 
            g_NewStringReady = true;
            g_BufferIndex = 0; // Reset index for the next message
        } 
        
        // 2. Otherwise, store the byte
        else if (g_BufferIndex < (MAX_STRING_LEN - 1)) 
        { 
            g_RxBuffer[g_BufferIndex] = received_byte;
         //   UARTprintf(&received_byte );
          //  UARTprintf("\n" );
            g_BufferIndex++;
        }
        
        // 3. Handle Overflow (optional, but good practice)
        else {
            // Buffer overflow, discard the current message and reset
            g_BufferIndex = 0; 
        }
    }
   //  UARTprintf("Received: %s\n", g_RxBuffer);
}


void toggleLed()
{
        // Turn on the BLUE LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        

       // UARTprintf("Hello, world!\n");
        //
        // Delay for a bit.
        //
        SysCtlDelay(SysCtlClockGet() / 10 / 3);

        //
        // Turn off the BLUE LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

        //
        // Delay for a bit.
        //
        SysCtlDelay(SysCtlClockGet() / 2/ 3);

}

// char UART_ReceiveChar_Polling(void) {
//     // Wait until the receive FIFO is not empty (UARTCharsAvail returns true)
//     while (!UARTCharsAvail(UART0_BASE)) {
//         // CPU is blocked here

//                 //
//         // Turn on the BLUE LED.
//         //
//         GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        
//        // UARTprintf("Hello, world!\n");
//         //
//         // Delay for a bit.
//         //
//         SysCtlDelay(SysCtlClockGet() / 10 / 3);

//         //
//         // Turn off the BLUE LED.
//         //
//         GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

//         //
//         // Delay for a bit.
//         //
//         SysCtlDelay(SysCtlClockGet() / 2/ 3);
//     }
    
//     // Read the character and return it
//     return (char)UARTCharGet(UART0_BASE);
// }