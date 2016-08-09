/*****************************************************************************
 * vim:sw=8:ts=8:si:et
 *
 * Title      : Microchip ENC28J60 Ethernet Interface Driver
 * Author     : Pascal Stang 
 * Modified by: Guido Socher, Vladimir Ustyugov, Andrey Chufyrev
 * Copyright  : LGPL V2 (http://www.gnu.org/licenses/old-licenses/lgpl-2.0.html)
 * Based on the file from AVRlib library by Pascal Stang.
 * For AVRlib See http://www.procyonengineering.com/
 * Used with explicit permission of Pascal Stang.
 *
 * This driver provides initialization and transmit/receive
 * functions for the Microchip ENC28J60 10Mb Ethernet Controller and PHY.
 ****************************************************************************/

#include "enc28j60.h"

#undef debug_out

static uint8_t Enc28j60Bank;
static int16_t gNextPacketPtr;

#define commandEmpty 0x00
uint32_t thrash;

#define CSACTIVE GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, (0<<6))  // PA6 - CS (SS)
#define CSPASSIVE GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, (1<<6))  // PA6 - CS (SS)


uint8_t enc28j60ReadOp(uint8_t op, uint8_t address)
{
	uint32_t data;

	// issue read command

#ifdef debug_out
	UARTprintf("### from ReadOp: (op | (address & ADDR_MASK)) = %x\n", (op | (address & ADDR_MASK)));
#endif
	CSACTIVE;
	SSIDataPut(SSI0_BASE,  (uint32_t) (op | (address & ADDR_MASK)));
	SSIDataGet(SSI0_BASE, &thrash);

	// read data
	SSIDataPut(SSI0_BASE, (uint32_t) commandEmpty);
	SSIDataGet(SSI0_BASE, &data);

#ifdef debug_out
	UARTprintf("### from ReadOp before dummy: received byte = %x\n", data);
#endif

	// do dummy read if needed (for mac and mii, see datasheet page 29)
	if(address & 0x80)
	{
		SSIDataPut(SSI0_BASE,  (uint32_t) commandEmpty);
		SSIDataGet(SSI0_BASE, &data);
	}
	CSPASSIVE;


#ifdef debug_out
	UARTprintf("### from ReadOp after dummy: received byte = %x\n", data);
#endif

	data &= 0x00FF;

	return((uint8_t)data);
}

void enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data)
{

	// issue write command
#ifdef debug_out
	UARTprintf("### from WriteOp: (op | (address & ADDR_MASK)) = %x\n", (op | (address & ADDR_MASK)));
#endif

	CSACTIVE;
	SSIDataPut(SSI0_BASE,  (uint32_t) (op | (address & ADDR_MASK)));
	SSIDataGet(SSI0_BASE, &thrash);

	// write data
#ifdef debug_out
	UARTprintf("### from WriteOp: transmitted byte = %x\n", data);
#endif
	SSIDataPut(SSI0_BASE,  (uint32_t) data);
	SSIDataGet(SSI0_BASE, &thrash);
	CSPASSIVE;

}

void enc28j60ReadBuffer(uint16_t len, uint8_t* rdata)
{

	uint32_t temp_data;
	// issue read command
#ifdef debug_out
	UARTprintf("### from ReadBuffer: command for RB = %x\n", ENC28J60_READ_BUF_MEM);
#endif

	CSACTIVE;

	SSIDataPut(SSI0_BASE,  (uint32_t) ENC28J60_READ_BUF_MEM);
	SSIDataGet(SSI0_BASE, &thrash);


	while(len)
	{
		len--;
		// read data
		SSIDataPut(SSI0_BASE,  (uint32_t) commandEmpty);
		SSIDataGet(SSI0_BASE, &temp_data);

#ifdef debug_out
		UARTprintf("### from ReadBuffer: received byte = %x\n", temp_data);
#endif
		temp_data &= 0x000000FF;
		*rdata = (uint8_t) temp_data;
		rdata++;
	}

	CSPASSIVE;

	*rdata='\0';

}

void enc28j60WriteBuffer(uint16_t len, uint8_t* wdata)
{

	// issue write command
#ifdef debug_out
	UARTprintf("### from WriteBuffer: command for WB = %x\n", ENC28J60_WRITE_BUF_MEM);
#endif

	CSACTIVE;

	SSIDataPut(SSI0_BASE,  (uint32_t) ENC28J60_WRITE_BUF_MEM);
	SSIDataGet(SSI0_BASE, &thrash);


	while(len)
	{
		len--;
		// write data
#ifdef debug_out
		UARTprintf("### from WriteBuffer: transmitted data = %x\n", (*wdata));
#endif
		SSIDataPut(SSI0_BASE,  (uint32_t) (*wdata));
		SSIDataGet(SSI0_BASE, &thrash);

		wdata++;
	}
	CSPASSIVE;

}

void enc28j60SetBank(uint8_t address)
{
        // set the bank (if needed)
        if((address & BANK_MASK) != Enc28j60Bank)
        {
                // set the bank
                enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
                enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
                Enc28j60Bank = (address & BANK_MASK);
        }
}

uint8_t enc28j60Read(uint8_t address)
{
        // set the bank
        enc28j60SetBank(address);
        // do the read
        return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}

// read upper 8 bits
uint8_t enc28j60PhyReadH(uint8_t address)
{

	// Set the right address and start the register read operation
	enc28j60Write(MIREGADR, address);
	enc28j60Write(MICMD, MICMD_MIIRD);
	_delay_us(10); // 40 us
	// wait until the PHY read completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY);

	// reset reading bit
	enc28j60Write(MICMD, 0x00);
	
	return (enc28j60Read(MIRDL));
}

void enc28j60Write(uint8_t address, uint8_t data)
{
        // set the bank
        enc28j60SetBank(address);
        // do the write
        enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

void enc28j60PhyWrite(uint8_t address, uint16_t data)
{
        // set the PHY register address
        enc28j60Write(MIREGADR, address);
        // write the PHY data
        enc28j60Write(MIWRL, data);
        enc28j60Write(MIWRH, data>>8);
        // wait until the PHY write completes
        while(enc28j60Read(MISTAT) & MISTAT_BUSY){
        	_delay_us(10); // 10 us
        }
}

void enc28j60clkout(uint8_t clk)
{
        //setup clkout: 2 is 12.5MHz:
	enc28j60Write(ECOCON, clk & 0x7);
}

void enc28j60Init(uint8_t* macaddr)
{

	// perform system reset
	enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
	_delay_us(20000); // 20 ms
	// check CLKRDY bit to see if reset is complete
        // The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
	//while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
	// do bank 0 stuff
	// initialize receive buffer
	// 16-bit transfers, must write low byte first
	// set receive buffer start address
	gNextPacketPtr = RXSTART_INIT;
        // Rx start
	enc28j60Write(ERXSTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXSTH, RXSTART_INIT>>8);
	// set receive pointer address
	enc28j60Write(ERXRDPTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXRDPTH, RXSTART_INIT>>8);
	// RX end
	enc28j60Write(ERXNDL, RXSTOP_INIT&0xFF);
	enc28j60Write(ERXNDH, RXSTOP_INIT>>8);
	// TX start
	enc28j60Write(ETXSTL, TXSTART_INIT&0xFF);
	enc28j60Write(ETXSTH, TXSTART_INIT>>8);
	// TX end
	enc28j60Write(ETXNDL, TXSTOP_INIT&0xFF);
	enc28j60Write(ETXNDH, TXSTOP_INIT>>8);
	// do bank 1 stuff, packet filter:
        // For broadcast packets we allow only ARP packtets
        // All other packets should be unicast only for our mac (MAADR)
        //
        // The pattern to match on is therefore
        // Type     ETH.DST
        // ARP      BROADCAST
        // 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
        // in binary these positions are:11 0000 0011 1111
        // This is hex 303F->EPMM0=0x3f,EPMM1=0x30
	enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
	enc28j60Write(EPMM0, 0x3f);
	enc28j60Write(EPMM1, 0x30);
	enc28j60Write(EPMCSL, 0xf9);
	enc28j60Write(EPMCSH, 0xf7);
        //
        //
	// do bank 2 stuff
	// enable MAC receive
	enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	// bring MAC out of reset
	enc28j60Write(MACON2, 0x00);
	// enable automatic padding to 60bytes and CRC operations
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
	// set inter-frame gap (non-back-to-back)
	enc28j60Write(MAIPGL, 0x12);
	enc28j60Write(MAIPGH, 0x0C);
	// set inter-frame gap (back-to-back)
	enc28j60Write(MABBIPG, 0x12);
	// Set the maximum packet size which the controller will accept
        // Do not send packets longer than MAX_FRAMELEN:
	enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);	
	enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);
	// do bank 3 stuff
        // write MAC address
        // NOTE: MAC address in ENC28J60 is byte-backward
        enc28j60Write(MAADR5, macaddr[0]);
        enc28j60Write(MAADR4, macaddr[1]);
        enc28j60Write(MAADR3, macaddr[2]);
        enc28j60Write(MAADR2, macaddr[3]);
        enc28j60Write(MAADR1, macaddr[4]);
        enc28j60Write(MAADR0, macaddr[5]);
	// no loopback of transmitted frames
	enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);
	// switch to bank 0
	enc28j60SetBank(ECON1);
	// enable interrutps
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
	// enable packet reception
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

// read the revision of the chip:
uint8_t enc28j60getrev(void)
{
        uint8_t rev;

        rev=enc28j60Read(EREVID);
        // microchip forgott to step the number on the silcon when they
        // released the revision B7. 6 is now rev B7. We still have
        // to see what they do when they release B8. At the moment
        // there is no B8 out yet
        if (rev>5) rev++;
	return(rev);
}

// A number of utility functions to enable/disable broadcast 
void enc28j60EnableBroadcast( void ) {
        uint8_t erxfcon;
        erxfcon=enc28j60Read(ERXFCON);
        erxfcon |= ERXFCON_BCEN;
        enc28j60Write(ERXFCON, erxfcon);
}

void enc28j60DisableBroadcast( void ) {
        uint8_t erxfcon;
        erxfcon=enc28j60Read(ERXFCON);
        erxfcon &= (0xff ^ ERXFCON_BCEN);
        enc28j60Write(ERXFCON, erxfcon);
}

// link status
uint8_t enc28j60linkup(void)
{
        // bit 10 (= bit 3 in upper reg)
        if (enc28j60PhyReadH(PHSTAT2) && 4){
                return(1);
        }
        return(0);
}

void enc28j60PacketSend(uint16_t len, uint8_t* packet)
{
        // Check no transmit in progress
        while (enc28j60ReadOp(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_TXRTS)
        {
                // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
                if( (enc28j60Read(EIR) & EIR_TXERIF) ) {
                        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
                        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
                }
        }
	// Set the write pointer to start of transmit buffer area
	enc28j60Write(EWRPTL, TXSTART_INIT&0xFF);
	enc28j60Write(EWRPTH, TXSTART_INIT>>8);
	// Set the TXND pointer to correspond to the packet size given
	enc28j60Write(ETXNDL, (TXSTART_INIT+len)&0xFF);
	enc28j60Write(ETXNDH, (TXSTART_INIT+len)>>8);
	// write per-packet control byte (0x00 means use macon3 settings)
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
	// copy the packet into the transmit buffer
	enc28j60WriteBuffer(len, packet);
	// send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}

// just probe if there might be a packet
uint8_t enc28j60hasRxPkt(void)
{
	if( enc28j60Read(EPKTCNT) ==0 ){
		return(0);
        }
        return(1);
}

// Gets a packet from the network receive buffer, if one is available.
// The packet will by headed by an ethernet header.
//      maxlen  The maximum acceptable length of a retrieved packet.
//      packet  Pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
uint16_t enc28j60PacketReceive(uint16_t maxlen, uint8_t* packet)
{
	uint16_t rxstat;
	uint16_t len;
	// check if a packet has been received and buffered
	//if( !(enc28j60Read(EIR) & EIR_PKTIF) )
        // The above does not work. See Rev. B4 Silicon Errata point 6.
	if( enc28j60Read(EPKTCNT) ==0 ){
		return(0);
        }

	// Set the read pointer to the start of the received packet
	enc28j60Write(ERDPTL, (gNextPacketPtr &0xFF));
	enc28j60Write(ERDPTH, (gNextPacketPtr)>>8);
	// read the next packet pointer
	gNextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	gNextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the packet length (see datasheet page 43)
	len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
        len-=4; //remove the CRC count
	// read the receive status (see datasheet page 43)
	rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	rxstat |= ((uint16_t)enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0))<<8;
	// limit retrieve length
        if (len>maxlen-1){
                len=maxlen-1;
        }
        // check CRC and symbol errors (see datasheet page 44, table 7-3):
        // The ERXFCON.CRCEN is set by default. Normally we should not
        // need to check this.
        if ((rxstat & 0x80)==0){
                // invalid
                len=0;
        }else{
                // copy the packet from the receive buffer
                enc28j60ReadBuffer(len, packet);
        }
	// Move the RX read pointer to the start of the next received packet
	// This frees the memory we just read out
	//enc28j60Write(ERXRDPTL, (gNextPacketPtr &0xFF));
	//enc28j60Write(ERXRDPTH, (gNextPacketPtr)>>8);
        //
        // Move the RX read pointer to the start of the next received packet
        // This frees the memory we just read out. 
        // However, compensate for the errata point 13, rev B4: never write an even address!
        // gNextPacketPtr is always an even address if RXSTOP_INIT is odd.
        if (gNextPacketPtr -1 > RXSTOP_INIT){ // RXSTART_INIT is zero, no test for gNextPacketPtr less than RXSTART_INIT.
                enc28j60Write(ERXRDPTL, (RXSTOP_INIT)&0xFF);
                enc28j60Write(ERXRDPTH, (RXSTOP_INIT)>>8);
        } else {
                enc28j60Write(ERXRDPTL, (gNextPacketPtr-1)&0xFF);
                enc28j60Write(ERXRDPTH, (gNextPacketPtr-1)>>8);
        }
	// decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
	return(len);
}

