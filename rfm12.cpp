#include "rfm12.h"
#include <avr/io.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <Arduino.h> 

/***********************************************************************
 * defines...
 **********************************************************************/
#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)

#define RFM_IRQ     8
#define SS_DDR      DDRB
#define SS_PORT     PORTB  // 
#define SS_BIT      1      // PB2, pin 3

#define SPI_SS      9     // PB1, pin 3
#define SPI_MISO    6     // PA6, pin 7
#define SPI_MOSI    5     // PA5, pin 8
#define SPI_SCK     4     // PA4, pin 9

/**
 * Pins: Attiny84
 */
//the interrupt mask register
#define RFM12_INT_MSK GIMSK

//the interrupt bit in the mask register
#define RFM12_INT_BIT (INT0)

#define RFM12_DEBUG 0

#else

/**
 * Pins: Arduino Uno
 */
#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      2     // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8

#define SPI_SS      10    // PB2, pin 16
#define SPI_MOSI    11    // PB3, pin 17
#define SPI_MISO    12    // PB4, pin 18
#define SPI_SCK     13    // PB5, pin 19

//the interrupt mask register
#define RFM12_INT_MSK EIMSK

//the interrupt bit in the mask register
#define RFM12_INT_BIT (INT0)

#endif 

#define RFM12_INT_ON() bitSet(RFM12_INT_MSK, RFM12_INT_BIT)
#define RFM12_INT_OFF() bitClear(RFM12_INT_MSK, RFM12_INT_BIT)    

/***********************************************************************
 * RFM12 command codes
 **********************************************************************/
#define RFM_RECEIVER_ON      0x82DD
#define RFM_TRANSMITTER_ON   0x823D
#define RFM_IDLE_MODE        0x820D
#define RFM_SLEEP_MODE       0x8205
#define RFM_WAKEUP_MODE      0x8207
#define RFM_TXREG_WRITE      0xB800
#define RFM_RX_FIFO_READ     0xB000
#define RFM_WAKEUP_TIMER     0xE000
#define RFM_READ_STATUS      0x0000
 
 // maximum transmit / receive buffer: 3 preamble, 2 sync, 3 header + data + 2 crc bytes + 2 tail bytes, see jeelib protocol
#define RFM_MAX   (RF12_MAXDATA + 12)

/***********************************************************************
 * the finite state machine
 **********************************************************************/
#define RFM12_IDLE                   0
#define RFM12_CAN_SEND               1
#define RFM12_TRANSMITTING_DATA      2
#define RFM12_TRANSMITTING_FINISHED  3
#define RFM12_RECEIVING              4
#define RFM12_DATA_RECEIVED          5

/***********************************************************************
 * status result of RFM12
 **********************************************************************/
#define RFM12_STATUS_RGIT 	0x8000
#define RFM12_STATUS_FFIT 	0x8000
#define RFM12_STATUS_POR 	0x4000
#define RFM12_STATUS_RGUR 	0x2000
#define RFM12_STATUS_FFOV 	0x2000
#define RFM12_STATUS_WKUP 	0x1000
#define RFM12_STATUS_EXT 	0x0800
#define RFM12_STATUS_LBD 	0x0400
#define RFM12_STATUS_FFEM 	0x0200
#define RFM12_STATUS_ATS 	0x0100
#define RFM12_STATUS_RSSI 	0x0100
#define RFM12_STATUS_DQD 	0x0080
#define RFM12_STATUS_CRL 	0x0040
#define RFM12_STATUS_ATGL 	0x0020


/// See http://blog.strobotics.com.au/2009/07/27/rfm12-tutorial-part-3a/
/// Transmissions are packetized, don't assume you can sustain these speeds! 
enum rfm12DataRates {
    RFM12_DATA_RATE_CMD = 0xC600,
    RFM12_DATA_RATE_9 = RFM12_DATA_RATE_CMD | 0x02,  // Approx 115200 bps
    RFM12_DATA_RATE_8 = RFM12_DATA_RATE_CMD | 0x05,  // Approx  57600 bps
    RFM12_DATA_RATE_7 = RFM12_DATA_RATE_CMD | 0x06,  // Approx  49200 bps
    RFM12_DATA_RATE_6 = RFM12_DATA_RATE_CMD | 0x08,  // Approx  38400 bps
    RFM12_DATA_RATE_5 = RFM12_DATA_RATE_CMD | 0x11,  // Approx  19200 bps
    RFM12_DATA_RATE_4 = RFM12_DATA_RATE_CMD | 0x23,  // Approx   9600 bps
    RFM12_DATA_RATE_3 = RFM12_DATA_RATE_CMD | 0x47,  // Approx   4800 bps
    RFM12_DATA_RATE_2 = RFM12_DATA_RATE_CMD | 0x91,  // Approx   2400 bps
    RFM12_DATA_RATE_1 = RFM12_DATA_RATE_CMD | 0x9E,  // Approx   1200 bps
    RFM12_DATA_RATE_DEFAULT = RFM12_DATA_RATE_7,
};

/***********************************************************************
 * Globals
 **********************************************************************/

/**
 * this struct contains all config value
 */
RFM12Var rfm12;

/**
 * theses global variables are accessed by the interrupt, so define them as volatile 
 */

/**
 * This is the buffer where we write the byte to transmit or the byte which we received
 */
volatile uint8_t rfm12Buffer[RFM_MAX];

/**
 * Current index within the buffer
 */
volatile uint8_t rfm12Index;

/**
 * the status of the finite state machine
 */
volatile uint8_t rfm12State;

/**
 * declaration for the compile
 */
void rfm12_feedback();

/***********************************************************************
 * Code
 **********************************************************************/
                
/**
 * initializes the spi and the interrupt pins...
 */
void rfm12_setup() {
                
  digitalWrite(SPI_SS, HIGH );    
  pinMode(SPI_SS, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_SCK, OUTPUT);
    
#ifdef SPCR    
    SPCR = _BV(SPE) | _BV(MSTR);
#if F_CPU > 10000000
    // SPI speed is < 2.5 Mhz, settings for the prescaler: clock / 2
    SPSR |= _BV(SPI2X);
#endif
#else
    // ATtiny
    USICR = bit( USIWM0 );
#endif

    // IRQ-Pin 
    pinMode( RFM_IRQ, INPUT );
    digitalWrite( RFM_IRQ, 1 );   // pull-up    
}

/**
 * Send byte throu spi 
 */
uint8_t rfm12_spi( uint8_t out ) {

#ifdef SPDR

    SPDR = out;
    // this loop spins 4 usec with a 2 MHz SPI clock    
    while(!(SPSR & _BV(SPIF))) ;
    return SPDR;
    
#else

    // ATtiny
    USIDR = out;
    byte v1 = bit(USIWM0) | bit(USITC);
    byte v2 = bit(USIWM0) | bit(USITC) | bit(USICLK);
    for (uint8_t i = 0; i < 8; ++i) {
        USICR = v1;
        USICR = v2;
    }
    return USIDR;
    
#endif

}

/**
 * writes a command to the rfm12 by spi
 */
uint16_t rfm12_cmd( uint16_t command ) {

#if F_CPU > 10000000
  // clock / 8 
  bitSet( SPCR, SPR0 );
#endif

  bitClear( SS_PORT, SS_BIT );  // Slave LOW  
  uint16_t result= rfm12_spi( command >> 8 ) << 8;
  result        |= rfm12_spi( command );
  bitSet( SS_PORT, SS_BIT );    // Slave HIGH

#if F_CPU > 10000000
  bitClear( SPCR, SPR0 );
#endif

  return result;
}

/**
 * initialize the RFM12 chip, 
 * the data structure for the RFM12. It also initialize the spi and attaches the interrupt handler.
 * @param id the node id 
 * @param band the band typical RF12_868MHZ in europe
 * @group the group id 0..255
 *
 * After this call the RFM12 chip sleeps
 */
void rfm12_setup( uint8_t id, uint8_t band, uint8_t group )  {
  
  rfm12State= RFM12_IDLE;
  rfm12.id   = id;
  rfm12.group= group;
    
  rfm12_setup();

  rfm12_cmd(RFM_READ_STATUS); // intitial SPI transfer added to avoid power-up problem

  rfm12_cmd(RFM_SLEEP_MODE); // DC (disable clk pin), enable lbd
    
  // wait until RFM12B is out of power-up reset, this takes several *seconds*
  rfm12_cmd(RFM_TXREG_WRITE); // in case we're still in OOK mode
  while( digitalRead(RFM_IRQ) == 0 )
      rfm12_cmd(RFM_READ_STATUS);
        
  rfm12_cmd(0x80C7 | (band << 4));   // EL (ena TX), EF (ena RX FIFO), 12.0pF 
  rfm12_cmd(0xA640);                 // 868MHz 
  rfm12_cmd(0xC606);                 // approx 49.2 Kbps, i.e. 10000/29/(1+6) Kbps
  rfm12_cmd(0x94A2);                 // VDI,FAST,134kHz,0dBm,-91dBm 
  rfm12_cmd(0xC2AC);                 // AL,!ml,DIG,DQD4 
  rfm12_cmd(0xCA83);                 // FIFO8,2-SYNC,!ff,DR 
  rfm12_cmd(0xCE00 | group);         // SYNC=2DXX； 
  rfm12_cmd(0xC483);                 // @PWR,NO RSTRIC,!st,!fi,OE,EN 
  rfm12_cmd(0x9850);                 // !mp,90kHz,MAX OUT 
  rfm12_cmd(0xCC77);                 // OB1，OB0, LPX,！ddy，DDIT，BW0 
  rfm12_cmd(0xE000);                 // NOT USE 
  rfm12_cmd(0xC800);                 // NOT USE 
  rfm12_cmd(0xC049);                 // 1.66MHz,3.1V 

#if RFM12_DEBUG
  Serial.println( "rfm12 initialized" );
#endif

  attachInterrupt( 0, rfm12_feedback, LOW );
}

/**
 * this is the interrupt function which is called by the RFM12 chip.
 */
void rfm12_feedback() {

  if (rfm12State == RFM12_RECEIVING) {

    uint8_t in = rfm12_cmd( RFM_RX_FIFO_READ );
    rfm12Buffer[ rfm12Index++ ]= in;
    // did we reache the end of message or buffer?
    if( rfm12Index >= rfm12Buffer[ 2 ] + 5 || rfm12Index >= RFM_MAX ) {      
      rfm12State= RFM12_DATA_RECEIVED;
      rfm12_cmd( RFM_IDLE_MODE );
    } // if 
  } // if 
  else
  // sending the data
  if( rfm12State == RFM12_TRANSMITTING_DATA ) {
    
    // write byte for byte
    rfm12_cmd( RFM_TXREG_WRITE | rfm12Buffer[ rfm12Index++ ] );
    // this helps for some reason to avoid freezing...
    rfm12_cmd( RFM_READ_STATUS );
    
    // out is the last byte?
    if( rfm12Index == rfm12Buffer[ 6 ] + 11 ) {    // data length + 9 
      rfm12State= RFM12_IDLE;
      rfm12_cmd( RFM_IDLE_MODE );
      rfm12_cmd( RFM_TXREG_WRITE | 0xAA );
    } // if 
  } // if 
  else {
    // keeps the 12fm running, otherwise the code will freeze...
    rfm12_cmd( RFM_READ_STATUS );    
  }
}

/**
 * updates the finite state machine. The client code tells the machine
 * that message was processed and the RFM12 should continue.
 */
void didProcess() {  
  if( rfm12State == RFM12_DATA_RECEIVED ) { rfm12State= RFM12_IDLE; }
}

/**
 * private function to start the receiver. It initialize the buffer.
 */
void _startReceiver() {
    RFM12_INT_OFF();    
    rfm12Buffer[ 0 ]= 0x00;        // hdr
    rfm12Buffer[ 1 ]= 0x00;        // length  
    rfm12Index= 0;                 // data from 2
    rfm12State= RFM12_RECEIVING;  // receiving mode    
    rfm12_cmd( RFM_RECEIVER_ON );      
    RFM12_INT_ON();  
}

/**
 * Starts the receiver mode. 
 */
void rfm12Listen() {  
  if( rfm12State == RFM12_IDLE ) { _startReceiver(); }
}

/**
 * check status and proof the message by crc...
 */
boolean didReceive() {

  if( rfm12State == RFM12_DATA_RECEIVED ) {
    uint16_t crc= 1;
    if( rfm12Buffer[ 1 ] < RFM_MAX ) {
      crc= _crc16_update( ~0, rfm12.group );
      for( uint8_t i= 0; i < rfm12Buffer[ 1 ] + 4; i++ ) {
        crc= _crc16_update( crc, rfm12Buffer[ i ] );
      } // for
    } // if 

#if RFM12_DEBUG
    for( uint8_t i= 0; i < rfm12Buffer[ 1 ] + 4; i++ ) {
        uint8_t v= rfm12Buffer[ i ] ;
        if( v < 16 )
          Serial.print( "0" );
          
        Serial.print( v, HEX );    
        Serial.print( " " );        
    } // for
    Serial.println( "" );    
    Serial.print( "CRC:"  );
    Serial.println( crc, HEX );    
#endif 

    // if crc wasn't correct we skip this message and listen again
    if( crc != 0 ) { _startReceiver(); }
    
    return crc == 0;
  } // if 
  return false;  
} // if

/**
 * checks if we are able to send. If the result is true then we have to send the message
 * because the function changed the status. therefore send the message otherwise the function will return false forever.
 */
boolean rfm12_canSend() {   

  RFM12_INT_OFF();
  uint8_t status= rfm12_cmd( RFM_READ_STATUS );
  RFM12_INT_ON();

#if RFM12_DEBUG  
  Serial.print( "STATUS:" );
  Serial.println( status, HEX );
  Serial.print( "RFM12_STATUS_RSSI:" );
  Serial.println( status & RFM12_STATUS_RSSI );
  Serial.print( "RFM12_STATUS_DQD:" );
  Serial.println( status & RFM12_STATUS_DQD );
#endif

  // only if we are idle and the rssi status flag is 0
  // rssi status flag is 1 means that someone other is transmitting, so we wait
  if( rfm12State == RFM12_IDLE && (status & RFM12_STATUS_RSSI) == 0 ) {
    rfm12_cmd( RFM_IDLE_MODE );
    rfm12State= RFM12_CAN_SEND;
    return true;
  }
  else
    return false;
}

/**
 * Sends a message
 * @param ptr  Pointer to the byte array
 * @param len  length in bytes
 */
void rfm12_send( const void* ptr, uint8_t len ) {

  RFM12_INT_OFF();
  // calc the crc check sum like the protocoll 2, that means with group id 
  uint16_t crc16= _crc16_update( ~0, rfm12.group );  

  // now we will fill the array  
  uint8_t i= 0;
  rfm12Buffer[ i++ ]= 0xAA;        // Preamble
  rfm12Buffer[ i++ ]= 0xAA;        // Preamble
  rfm12Buffer[ i++ ]= 0xAA;        // Preamble
  rfm12Buffer[ i++ ]= 0x2D;        // Sync1
  rfm12Buffer[ i++ ]= rfm12.group; // Sync2   rfm12Buffer[ 4 ]
  
  // header: the node id
  rfm12Buffer[ i ]= rfm12.id;  
  crc16= _crc16_update( crc16, rfm12Buffer[ i++ ] ); 
  
  // length at rfm12Buffer[ 6 ]
  rfm12Buffer[ i ]= len;
  crc16= _crc16_update( crc16, rfm12Buffer[ i++ ] ); 
  
  // now copy the payload to the buffer begins from rfm12Buffer[ 7 ]
  for( uint8_t j= 0; j < len; j++ ) {
    rfm12Buffer[ i ]= ((uint8_t *)ptr)[ j ];
    crc16= _crc16_update( crc16, rfm12Buffer[ i++ ] ); 
  } // for
  
  // crc in little endian format...
  rfm12Buffer[ i++ ]= crc16;
  rfm12Buffer[ i++ ]= crc16 >> 8;

  rfm12Buffer[ i++ ]= 0x10;    // We are sending two additional bytes.
  rfm12Buffer[ i++ ]= 0x20;    // Without them the crc is often wrong, but the message was correct. I think the FIFO needs to be filled

  // now enter the transmitter mode and the interrupt routine will be called  
  rfm12State= RFM12_TRANSMITTING_DATA;
  rfm12Index = 0;  

  
#if RFM12_DEBUG
  for( uint8_t j= 0; j < i; j++ ) {
    uint8_t v= rfm12Buffer[ j ];
    if( v < 16 )
    Serial.print( 0 );
    Serial.print( v, HEX );    
    Serial.print( " " );        
  } // for 
  
  Serial.println( "" );
  delay( 100 );
#endif 
  
  RFM12_INT_ON();
  rfm12_cmd( RFM_TRANSMITTER_ON );
}

/**
 * put the RFM12 to sleep and save some power
 */
void rfm12_sleep() { 
  RFM12_INT_OFF();
  rfm12_cmd( RFM_SLEEP_MODE );
  RFM12_INT_ON();
}

/**
 * wakes up the RFM12
 */
void rfm12_wakeup() { 
  RFM12_INT_OFF();
  rfm12_cmd( RFM_IDLE_MODE );
  RFM12_INT_ON();
}




