# pic32fatfs
PIC32 FatFs implementation optimised for reads

This is a modified version of the PIC32 FatFs originally implemented by Aiden Morrison to now make full use of the SPI bus when reading from SD cards with slower PIC32 devices.

On a 40MHz PIC32MX running a 20MHz SPI bus on 4K aligned reads one should now be able to achieve 90%+ bus utilization. Overall this translates into a performance increase of 80% over the original code.

Implementing this required using the PIC32's SPI controller in Enhanced Buffer mode, allowing multiple writes to be queued in advance on the HW 128-bit FIFO. Combined with a switch to 32-bit transfers, this sufficiently reduced the cycles required and removed the significant bus pauses that were present before.
