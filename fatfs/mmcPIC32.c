/*------------------------------------------------------------------------/
/  MMCv3/SDv1/SDv2 (in SPI mode) control module
/-------------------------------------------------------------------------/
/
/  Copyright (C) 2010, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/
//Edited by A. Morrison to function on PIC32.

// Ported by Riccardo Leonardi to PIC32MX795F512L  (22/11/2011)
// Many thanks to Aiden Morrison's good work!
// changes: parametrization of SPI port number

// Modified by Bryn Thomas (11/09/2016) to use Enhanced Buffer SPI mode
// and boost read performance with 32-bit transfers

#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _SUPPRESS_PLIB_WARNING

#include <p32xxxx.h>
#include <plib.h>
#include "diskio.h"

/* Definitions for MMC/SDC command */
#define CMD0   (0)			/* GO_IDLE_STATE */
#define CMD1   (1)			/* SEND_OP_COND */
#define ACMD41 (41|0x80)	/* SEND_OP_COND (SDC) */
#define CMD8   (8)			/* SEND_IF_COND */
#define CMD9   (9)			/* SEND_CSD */
#define CMD10  (10)			/* SEND_CID */
#define CMD12  (12)			/* STOP_TRANSMISSION */
#define ACMD13 (13|0x80)	/* SD_STATUS (SDC) */
#define CMD16  (16)			/* SET_BLOCKLEN */
#define CMD17  (17)			/* READ_SINGLE_BLOCK */
#define CMD18  (18)			/* READ_MULTIPLE_BLOCK */
#define CMD23  (23)			/* SET_BLOCK_COUNT */
#define ACMD23 (23|0x80)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24  (24)			/* WRITE_BLOCK */
#define CMD25  (25)			/* WRITE_MULTIPLE_BLOCK */
#define CMD41  (41)			/* SEND_OP_COND (ACMD) */
#define CMD55  (55)			/* APP_CMD */
#define CMD58  (58)			/* READ_OCR */


/* Port Controls  (Platform dependent) */
/*
#define CS_SETOUT() TRISFbits.TRISF12 = 0 
#define CS_LOW()  _LATF12 = 0	//MMC CS = L
#define CS_HIGH() _LATF12 = 1	//MMC CS = H
//Change the SPI port number as needed on the following 5 lines
#define SPIBRG  SPI4BRG
#define SPIBUF  SPI4BUF
#define SPISTATbits SPI4STATbits
#define SPI_CHANNEL SPI_CHANNEL4
#define SPICONbits SPI4CONbits
*/
//EXPLORER 16 CONFIGURATION
#define CS_SETOUT() TRISBbits.TRISB1 = 0 
#define CS_LOW()  LATBbits.LATB1 = 0	//MMC CS = L
#define CS_HIGH() LATBbits.LATB1 = 1	//MMC CS = H
//Change the SPI port number as needed on the following 5 lines
#define SPIBRG  SPI1BRG
#define SPIBUF  SPI1BUF
#define SPISTATbits SPI1STATbits
#define SPI_CHANNEL SPI_CHANNEL1
#define SPICONbits SPI1CONbits

//REVEIW THIS LATER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Makes assumptions that sockwp and sockins are on the same port...
// Should probably remove sockport define and then go fix what used it to be general.
#define SOCKPORT	PORTA		/* Socket contact port */
#define SOCKWP	0 // disable write protect (1<<10)		/* Write protect switch (RB10) */
#define SOCKINS	0 // Pretend card is always inserted (1<<11)	/* Card detect switch (RB11) */

#define	FCLK_SLOW()	SPIBRG = 64		/* Set slow clock (100k-400k) */
#define	FCLK_FAST()	SPIBRG = 0		/* Set fast clock (depends on the CSD) */



/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

static volatile
DSTATUS Stat = STA_NOINIT;	/* Disk status */

static volatile
UINT Timer1, Timer2;		/* 1000Hz decrement timer */

static
UINT CardType;



/*-----------------------------------------------------------------------*/
/* Exchange a byte between PIC and MMC via SPI  (Platform dependent)     */
/*-----------------------------------------------------------------------*/

// Regular receive and transmit functions changed to use
// Enhanced Buffer semantics to reduce unnecessary mode changes

#define xmit_spi(dat) 	xchg_spi(dat)
#define rcvr_spi()		xchg_spi(0xFF)
//#define rcvr_spi_m(p)	SPI2BUF = 0xFF; while (!SPI2STATbits.SPIRBF); *(p) = (BYTE)SPI2BUF;
#define rcvr_spi_m(p)	SPIBUF = 0xFF; while (SPISTATbits.SPIRBE); *(p) = (BYTE)SPIBUF;

static
BYTE xchg_spi (BYTE dat)
{
//	SPI2BUF = dat;
//	while (!SPI2STATbits.SPIRBF);
//	return (BYTE)SPI2BUF;
	SPIBUF = dat;
	while (SPISTATbits.SPIRBE);
	return (BYTE)SPIBUF;
}

static BYTE SPI_transaction_in_progress = 0;

static int select(void);
static void deselect(void);
static void spi_init(void);
static void spi_commit(void);
static void spi_go_8(void);
static void spi_go_32(void);

// Buffers used to translate between the 32-bit chunks retrieved
// and the 8-bit data required

typedef union {
    UINT raw_data[8];
    BYTE transformed[32];
} spi_union;

static spi_union virtual_spi_rx;
static BYTE virtual_spi_rx_count;
static BYTE virtual_spi_rx_read_pos;
static BYTE virtual_spi_rx_write_pos;

static spi_union virtual_spi_tx;
static INT virtual_spi_tx_count;
static BYTE virtual_spi_tx_read_pos;
static BYTE virtual_spi_tx_write_pos;

// Used to maintain compatibility with existing unchanged functions
// and handle the transitions between working with them and working
// with the new 32-bit read functions

static void end_active_SPI_transaction(void){
    if (SPI_transaction_in_progress){
        spi_go_8();
        deselect();
        SPI_transaction_in_progress = 0;
    }
}

static void begin_SPI_transaction(void){
    if (!SPI_transaction_in_progress){
        select();
        SPI_transaction_in_progress = 1;
        spi_init();
        spi_go_32();
    }
}

// Transfers between the 32-bit system SPI buffers and the virtual
// buffers that give easier access to 8-bit data

static inline void read_spi_from_system(void){
    BYTE count;
    count = SPISTATbits.RXBUFELM;
    while (count > 0) {
        if (virtual_spi_rx_count > 27) return;
        count--;
        virtual_spi_rx_write_pos = (virtual_spi_rx_write_pos - 4) & 0x1F;
        virtual_spi_rx.raw_data[virtual_spi_rx_write_pos >> 2] = SPIBUF;
        virtual_spi_rx_count = virtual_spi_rx_count + 4;
    }
}

static inline void send_spi_to_system(void){
    BYTE count;
    if (virtual_spi_rx_count > 15) return;
    count = SPISTATbits.TXBUFELM;
    while (virtual_spi_tx_count > 3) {
        count++;
        if (count > 2) return; 
        virtual_spi_tx_read_pos = (virtual_spi_tx_read_pos - 4) & 0x1F;
        SPIBUF = virtual_spi_tx.raw_data[virtual_spi_tx_read_pos >> 2];
        virtual_spi_tx_count = virtual_spi_tx_count - 4;
    }
}

// Switches between 8-bit and 32-bit modes. Clears hardware buffers first.

static void spi_go_8(void) {
    send_spi_to_system();
    while ((SPISTATbits.SPIBUSY) || (!SPISTATbits.SPIRBE) || (!SPISTATbits.SPITBE)){read_spi_from_system();};
    SPICONbits.MODE32 = 0;    
}

static void spi_go_32(void) {
    while ((SPISTATbits.SPIBUSY) || (!SPISTATbits.SPITBE)) {};
    SPICONbits.MODE32 = 1;
}

#define read_spi_byte() virtual_spi_rx.transformed[--virtual_spi_rx_read_pos & 0x1F]; virtual_spi_rx_count--;
#define send_spi_byte(this_byte)  virtual_spi_tx_write_pos--;virtual_spi_tx.transformed[virtual_spi_tx_write_pos & 0x1F]=this_byte; virtual_spi_tx_count++;

#define UNROLL_2(X) X X
#define UNROLL_4(X) UNROLL_2(X) UNROLL_2(X)
#define UNROLL_8(X) UNROLL_4(X) UNROLL_4(X)

// Resets the virtual buffers to their default state

static void spi_init(){
    virtual_spi_rx_count = 0;
    virtual_spi_rx_read_pos = 0;
    virtual_spi_rx_write_pos = 0;    

    virtual_spi_tx_count = 0;
    virtual_spi_tx_read_pos = 0;
    virtual_spi_tx_write_pos = 0; 
}

// Populates the virtual transmit buffers with 0xFF, which is
// sent while waiting for a response from the slave device.

static void spi_fill_send_cmd(void){
    BYTE count;
    for (count = 0;count < 32;count++){
        virtual_spi_tx.transformed[count]=0xFF;
    };
}

// Flushes all lingering data from the virtual buffers out the SPI bus
// Switches to 8-bit to make sure it can clear out all stragglers

static void spi_commit(void){
    spi_go_8();
    while ((SPISTATbits.SPIBUSY)){};
    while (virtual_spi_tx_count > 0){
        virtual_spi_tx_read_pos = (virtual_spi_tx_read_pos - 1) & 0x1F;
        SPIBUF = virtual_spi_tx.transformed[virtual_spi_tx_read_pos];
        virtual_spi_tx_count--;
    }
    while ((SPISTATbits.SPIBUSY) || (!SPISTATbits.SPITBE)) {};
    while (!SPISTATbits.SPIRBE) {
        virtual_spi_rx_write_pos = (virtual_spi_rx_write_pos - 1) & 0x1F;
        virtual_spi_rx.transformed[virtual_spi_rx_write_pos] = SPIBUF;
        virtual_spi_rx_count++;       
    }
    spi_go_32();
}

// A 32-bit compatible form of the rcvr_datablock function
// This one however is only used for reading sectors, hence the
// requirement of a multiple of 512

static
int rcvr_datablock32 (	/* 1:OK, 0:Failed */
	BYTE *buff,			/* Data buffer to store received data */
	INT btr			/* Byte count (must be multiple of 512) */
)
{
  	BYTE token;
    INT total_recv;
    INT small_btr;
    INT sectors_left;
    INT donezo;
    
    spi_fill_send_cmd();
    small_btr = 512;
    sectors_left = btr >> 9;
    // Data could have already been sent to the queue as part of the initial
    // command that began the transfer.
    // We make sure we take account of it when working out how many more SPI transactions
    // are required.
    // Since we know roughly how much data we will need to submit to the card, we preload
    // the total transfer size and later in the program add or subtract from this value
    // as needed
    virtual_spi_tx_count = btr + sectors_left * 2 - (virtual_spi_rx_count + SPISTATbits.RXBUFELM*4);

    total_recv = 0;

    do {
        sectors_left--;
        do {							
            read_spi_from_system();
            send_spi_to_system();
            if (virtual_spi_rx_count > 3){
                // Wait for the card to signal to us that it's ready
                UNROLL_4 (virtual_spi_tx_count++; token = read_spi_byte(); if (token != 0xFF) break;)
            };
    	} while (1);

        // We expect the data to begin with a 0xFE signal
        if(token != 0xFE) return 0;		/* If not valid data token, retutn with error */

        if (sectors_left == 0) small_btr = small_btr - 16;

        while ((total_recv < small_btr)) {
            // This is the most performance sensitive part of this system
            // Just one or two extra cycles needed to process instructions can
            // mean one isn't able to use the full bus speed.
            
            // This means that instead of using the "send_spi_byte" helper
            // we need to directly load up SPIBUF and adjust the virtual_spi_tx_count.
            // It does mean that the relationship between tx_read_pos and tx_write_pos
            // gets messed up, so before we write real values to the system we will
            // need to perform an spi_init.
            
            read_spi_from_system();
            if (virtual_spi_rx_count > 7) {
                if ((virtual_spi_tx_count > 7) && !(virtual_spi_rx_count > 15)){
                    SPIBUF=0xFFFFFFFF;
                    SPIBUF=0xFFFFFFFF;
                    virtual_spi_tx_count -= 8;
                }
                UNROLL_8(*(buff++) = read_spi_byte();)
                total_recv = total_recv + 8;
                continue;
            }
            send_spi_to_system();
        };
        if (sectors_left == 0) break;
        
        small_btr = small_btr+512;

        while (virtual_spi_rx_count < 4) read_spi_from_system();
        UNROLL_2(read_spi_byte();)
    } while (sectors_left > 0);
    
    donezo = 0;
    
    // Since there is absolutely no guarantee that the number of bytes we
    // actually read will be a multiple of 4, we need to switch out of 32-bit
    // mode and make sure we pick up the last of the stragglers.

	while ((total_recv < btr)) {							
        if ((virtual_spi_rx_count > 0)) {*(buff++) = read_spi_byte();total_recv++;};
        if (!donezo){
            read_spi_from_system();
            send_spi_to_system();
            if ((virtual_spi_tx_count < 4)) {    
                spi_commit();
                donezo = 1;
            };
        };
	};

    spi_commit();
    
    // Read the last two CRC bytes and ignore them
    read_spi_byte();
    read_spi_byte();
    spi_init();
    
	return 1;						/* Return with success */
}

// 32-bit version of the send_cmd function supporting a limited
// subset of reading instructions, needed to support disk_read.

static
BYTE send_cmd32 (
	BYTE cmd,		/* Command byte */
	DWORD arg		/* Argument */
)
{
	BYTE res;
    INT countdown;
 
    read_spi_from_system();
    while (virtual_spi_rx_count > 0) {read_spi_byte();};

    send_spi_byte(0x40 | cmd);
    send_spi_byte((BYTE)(arg >> 24));
    send_spi_byte((BYTE)(arg >> 16));
    send_spi_byte((BYTE)(arg >> 8));
    send_spi_byte((BYTE)arg);
    send_spi_byte(0x01);
    send_spi_byte(0xFF);
    send_spi_byte(0xFF);
    send_spi_to_system();
    while (virtual_spi_rx_count < 8) read_spi_from_system();
    UNROLL_4(read_spi_byte();)
    UNROLL_2(read_spi_byte();)
    
    if (cmd == CMD12) read_spi_byte();

    countdown = 30;
    do {
        UNROLL_4(send_spi_byte(0xFF);)
        send_spi_to_system();
        while (virtual_spi_rx_count < 4) read_spi_from_system();
        UNROLL_4(res = read_spi_byte(); if (!(res & 0x80)) break;)
    } while (--countdown > 0);
    spi_commit();
    return res;

}

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
BYTE wait_ready (void)
{
	BYTE res;


	Timer2 = 500;	/* Wait for ready in timeout of 500ms */
	rcvr_spi();
	do
		res = rcvr_spi();
	while ((res != 0xFF) && Timer2);

	return res;
}



/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/

static
void deselect (void)
{
	CS_HIGH();
	rcvr_spi();
}



/*-----------------------------------------------------------------------*/
/* Select the card and wait ready                                        */
/*-----------------------------------------------------------------------*/

static
int select (void)	/* 1:Successful, 0:Timeout */
{
	CS_LOW();
	if (wait_ready() != 0xFF) {
		deselect();
		return 0;
	}
	return 1;
}




/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,				/* Physical drive nmuber (0) */
	const BYTE *buff,		/* Pointer to the data to be written */
	DWORD sector,			/* Start sector number (LBA) */
	BYTE count				/* Sector count (1..255) */
)
{
    end_active_SPI_transaction();
    
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	if (count == 1) {		/* Single block write */
		if ((send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple block write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY */






/*-----------------------------------------------------------------------*/
/* Power Control  (Platform dependent)                                   */
/*-----------------------------------------------------------------------*/
/* When the target system does not support socket power control, there   */
/* is nothing to do in these functions and chk_power always returns 1.   */

static
void power_on (void)
{
						/* Setup SPI2 */
						/* Setup SPI */
	// Setup CS as output
	CS_SETOUT();
	// configured for ~400 kHz operation - reset later to 20 MHz
//	SpiChnOpen(SPI_CHANNEL2,SPI_OPEN_MSTEN|SPI_OPEN_CKP_HIGH|SPI_OPEN_SMP_END|SPI_OPEN_MODE8,64); 
	SpiChnOpen(SPI_CHANNEL,SPI_OPEN_MSTEN|SPI_OPEN_CKP_HIGH|SPI_OPEN_SMP_END|SPI_OPEN_MODE8,64); 
//	SPI2CONbits.ON = 1;
    // Enables SPI port to run in Enhanced Buffer mode
    SPICONbits.ENHBUF = 1;
	SPICONbits.ON = 1;
}

static
void power_off (void)
{
	select();			/* Wait for card ready */
	deselect();

//	SPI2CONbits.ON = 0;			/* Disable SPI2 */
	SPICONbits.ON = 0;			/* Disable SPI */

	Stat |= STA_NOINIT;	/* Set STA_NOINIT */
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static
int rcvr_datablock (	/* 1:OK, 0:Failed */
	BYTE *buff,			/* Data buffer to store received data */
	UINT btr			/* Byte count (must be multiple of 4) */
)
{
	BYTE token;


	Timer1 = 100;
	do {							/* Wait for data packet in timeout of 100ms */
		token = rcvr_spi();
	} while ((token == 0xFF) && Timer1);

	if(token != 0xFE) return 0;		/* If not valid data token, retutn with error */

	do {							/* Receive the data block into buffer */
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
	} while (btr -= 4);
	rcvr_spi();						/* Discard CRC */
	rcvr_spi();

	return 1;						/* Return with success */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
static
int xmit_datablock (	/* 1:OK, 0:Failed */
	const BYTE *buff,	/* 512 byte data block to be transmitted */
	BYTE token			/* Data token */
)
{
	BYTE resp;
	UINT bc = 512;

    end_active_SPI_transaction();
    
	if (wait_ready() != 0xFF) return 0;

	xmit_spi(token);		/* Xmit a token */
	if (token != 0xFD) {	/* Not StopTran token */
		do {						/* Xmit the 512 byte data block to the MMC */
			xmit_spi(*buff++);
			xmit_spi(*buff++);
		} while (bc -= 2);
		xmit_spi(0xFF);				/* CRC (Dummy) */
		xmit_spi(0xFF);
		resp = rcvr_spi();			/* Receive a data response */
		if ((resp & 0x1F) != 0x05)	/* If not accepted, return with error */
			return 0;
	}

	return 1;
}
#endif	/* _READONLY */



/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (
	BYTE cmd,		/* Command byte */
	DWORD arg		/* Argument */
)
{
	BYTE n, res;

    end_active_SPI_transaction();
    
	if (cmd & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card and wait for ready */
	deselect();
	if (!select()) return 0xFF;

	/* Send command packet */
	xmit_spi(0x40 | cmd);			/* Start + Command index */
	xmit_spi((BYTE)(arg >> 24));	/* Argument[31..24] */
	xmit_spi((BYTE)(arg >> 16));	/* Argument[23..16] */
	xmit_spi((BYTE)(arg >> 8));		/* Argument[15..8] */
	xmit_spi((BYTE)arg);			/* Argument[7..0] */
	n = 0x01;						/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;		/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;		/* Valid CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive command response */
	if (cmd == CMD12) rcvr_spi();	/* Skip a stuff byte when stop reading */
	n = 10;							/* Wait for a valid response in timeout of 10 attempts */
	do
		res = rcvr_spi();
	while ((res & 0x80) && --n);

	return res;			/* Return with the response value */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE drv		/* Physical drive nmuber (0) */
)
{
	BYTE n, cmd, ty, ocr[4];

    end_active_SPI_transaction();

	if (drv) return STA_NOINIT;			/* Supports only single drive */
	if (Stat & STA_NODISK) return Stat;	/* No card in the socket */

	power_on();							/* Force socket power on */
	FCLK_SLOW();
	for (n = 10; n; n--) rcvr_spi();	/* 80 dummy clocks */

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
		Timer1 = 1000;						/* Initialization timeout of 1000 msec */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
			for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();			/* Get trailing return value of R7 resp */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* The card can work at vdd range of 2.7-3.6V */
				while (Timer1 && send_cmd(ACMD41, 0x40000000));	/* Wait for leaving idle state (ACMD41 with HCS bit) */
				if (Timer1 && send_cmd(CMD58, 0) == 0) {			/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();
					ty = (ocr[0] & 0x40) ? CT_SD2|CT_BLOCK : CT_SD2;	/* SDv2 */
				}
			}
		} else {							/* SDv1 or MMCv3 */
			if (send_cmd(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
			}
			while (Timer1 && send_cmd(cmd, 0));		/* Wait for leaving idle state */
			if (!Timer1 || send_cmd(CMD16, 512) != 0)	/* Set read/write block length to 512 */
				ty = 0;
		}
	}
	CardType = ty;
	deselect();

	if (ty) {			/* Initialization succeded */
		Stat &= ~STA_NOINIT;	/* Clear STA_NOINIT */
		FCLK_FAST();
	} else {			/* Initialization failed */
		power_off();
	}

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0) */
)
{
	if (drv) return STA_NOINIT;		/* Supports only single drive */
	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0) */
	BYTE *buff,		/* Pointer to the data buffer to store read data */
	DWORD sector,	/* Start sector number (LBA) */
	BYTE count		/* Sector count (1..255) */
)
{
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
    
    begin_SPI_transaction();

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	if (count == 1) {		/* Single block read */
		if ((send_cmd32(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
			&& rcvr_datablock32(buff, 512))
			count = 0;
	}
	else {				/* Multiple block read */
		if (send_cmd32(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
			if (rcvr_datablock32(buff, count*512)) count = 0;
			send_cmd32(CMD12, 0);				/* STOP_TRANSMISSION */
		}
	}

	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT w (
	BYTE drv,				/* Physical drive nmuber (0) */
	const BYTE *buff,		/* Pointer to the data to be written */
	DWORD sector,			/* Start sector number (LBA) */
	BYTE count				/* Sector count (1..255) */
)
{
    end_active_SPI_transaction();
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	if (count == 1) {		/* Single block write */
		if ((send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple block write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive data block */
)
{
	DRESULT res;
	BYTE n, csd[16], *ptr = buff;
	DWORD csize;

    end_active_SPI_transaction();
    
	if (drv) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	res = RES_ERROR;
	switch (ctrl) {
		case CTRL_SYNC :	/* Flush dirty buffer if present */
			if (select()) {
				deselect();
				res = RES_OK;
			}
			break;

		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (WORD) */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
				if ((csd[0] >> 6) == 1) {	/* SDv2? */
					csize = csd[9] + ((WORD)csd[8] << 8) + 1;
					*(DWORD*)buff = (DWORD)csize << 10;
				} else {					/* SDv1 or MMCv2 */
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
					*(DWORD*)buff = (DWORD)csize << (n - 9);
				}
				res = RES_OK;
			}
			break;

		case GET_SECTOR_SIZE :	/* Get sectors on the disk (WORD) */
			*(WORD*)buff = 512;
			res = RES_OK;
			break;

		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sectors (DWORD) */
			if (CardType & CT_SD2) {	/* SDv2? */
				if (send_cmd(ACMD13, 0) == 0) {		/* Read SD status */
					rcvr_spi();
					if (rcvr_datablock(csd, 16)) {				/* Read partial block */
						for (n = 64 - 16; n; n--) rcvr_spi();	/* Purge trailing data */
						*(DWORD*)buff = 16UL << (csd[10] >> 4);
						res = RES_OK;
					}
				}
			} else {					/* SDv1 or MMCv3 */
				if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
					if (CardType & CT_SD1) {	/* SDv1 */
						*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
					} else {					/* MMCv3 */
						*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
					}
					res = RES_OK;
				}
			}
			break;

		case MMC_GET_TYPE :		/* Get card type flags (1 byte) */
			*ptr = CardType;
			res = RES_OK;
			break;

		case MMC_GET_CSD :	/* Receive CSD as a data block (16 bytes) */
			if ((send_cmd(CMD9, 0) == 0)	/* READ_CSD */
				&& rcvr_datablock(buff, 16))
				res = RES_OK;
			break;

		case MMC_GET_CID :	/* Receive CID as a data block (16 bytes) */
			if ((send_cmd(CMD10, 0) == 0)	/* READ_CID */
				&& rcvr_datablock(buff, 16))
				res = RES_OK;
			break;

		case MMC_GET_OCR :	/* Receive OCR as an R3 resp (4 bytes) */
			if (send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
				for (n = 0; n < 4; n++)
					*((BYTE*)buff+n) = rcvr_spi();
				res = RES_OK;
			}
			break;

		case MMC_GET_SDSTAT :	/* Receive SD statsu as a data block (64 bytes) */
			if (send_cmd(ACMD13, 0) == 0) {	/* SD_STATUS */
				rcvr_spi();
				if (rcvr_datablock(buff, 64))
					res = RES_OK;
			}
			break;

		default:
			res = RES_PARERR;
	}

	deselect();

	return res;
}


