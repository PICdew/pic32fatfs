//Wrapper for PIC32 FatFS implementation
//FatFS available from http://elm-chan.org/fsw/ff/00index_e.html
//Adapted by Aiden Morrison, 2011.


// Ported by Riccardo Leonardi to PIC32MX795F512L  (22/11/2011)
// Many thanks to Aiden Morrison's good work!
// Changes from Aiden Morrison version:
// -> device changed to PIC32MX795F512L (used into chipKIT Max32)
// -> Header files included via "Include Search Path" option in project
// -> System config performance optimization
// -> LEDs removed (I don't have them in my "shield")
// -> FatFs version updated to R0.09 September 6, 2011 (multi-partition)
// -> get_fattime routine moved to main
// -> disk_timerproc call added (handled by ISR) 
// -> parametrization of SPI port number
// -> core_timer added for rtc and benchmarking purposes

#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _SUPPRESS_PLIB_WARNING
#define GetSystemClock() (40000000ul)

#define TOGGLES_PER_SEC			1000
#define CORE_TICK_RATE	       (GetSystemClock()/2/TOGGLES_PER_SEC)

#include <p32xxxx.h>

#include <plib.h>
#include ".\fatfs\ff.h"
#include <stdint.h>

#pragma config   JTAGEN    = OFF    // JTAG Enable OFF
#pragma config   FNOSC     = FRCPLL // Fast RC w PLL 8mHz internal rc Osc
#pragma config   FPLLIDIV  = DIV_2  // PLL in 8mHz/2 = 4mHz
#pragma config   FPLLMUL   = MUL_20 // PLL mul 4mHz * 20 = 80mHz 24??
#pragma config   FPLLODIV  = DIV_2  // PLL Out 8mHz/2= 40 mHz system frequency osc
#pragma config   FPBDIV    = DIV_2  // Peripheral Bus Divisor
#pragma config   FCKSM     = CSECME // Clock Switch Enable, FSCM Enabled
#pragma config   POSCMOD   = OFF    // Primary osc disabled
#pragma config   IESO      = OFF    // Internal/external switch over
#pragma config   OSCIOFNC  = OFF    // CLKO Output Signal Active on the OSCO Pin
#pragma config   FWDTEN    = OFF    // Watchdog Timer Enable:
#pragma config   FSOSCEN   = OFF

FATFS Fatfs;
FIL file1;
INT bytes_read;
BYTE bbuffer[4096];

int count_1;
int count_2;

int main (void)
{
    // Enable optimal performance
    INTEnableSystemMultiVectoredInt();
    SYSTEMConfigPerformance(GetSystemClock());
    mOSCSetPBDIV(OSC_PB_DIV_1);				// Use 1:1 CPU Core:Peripheral clocks
    
    // Map the Peripheral Pin Select registers for SPI in/out

    SDI1R = (SDI1R & 0xF0) | 0b0100;
    RPB11R = (RPB11R & 0xF0) | 0b0011;

    TRISB = 0b100000000;

    // Turn on busy LED
    
    PORTBbits.RB7 = 1;
    
    // Initialize Disk

    while (disk_initialize(0));

    // Mount Filesystem

    f_mount(0, &Fatfs);

    FRESULT FOpenRes;
    char file1name[]= "thesign.mod";
    FOpenRes=f_open(&file1, file1name, (FA_OPEN_EXISTING | FA_READ));
    
    // Read 32MB of data from the file

    for (count_1=0;count_1<64;count_1++) {
        f_lseek(&file1, 0);
        for (count_2=0;count_2<128;count_2++) {
            FOpenRes=f_read (&file1, bbuffer, 4096, &bytes_read);
        }
    };

    f_close(&file1);

    // Unmount filesystem

    f_mount(0,NULL);
    PORTBbits.RB7 = 0;

	Nop();
	while(1);
}
