/**
 *
 * Alternative implemention for the RFM12B chip
 * the message is like the JeeNode message: header, length, data, crc
 * 
 */
#include <Arduino.h> 
#include <interrupt.h>
#include <io.h>

/**
 * to debug set this to 1
 * it will produce serial messages
 */
#define RFM12_DEBUG     1


// RFM12 Maximum message size in bytes.
#define RF12_MAXDATA    66

// 
#define RF12_433MHZ     1
#define RF12_868MHZ     2
#define RF12_915MHZ     3

/**
 * define the run time vars
 */
typedef struct {  
  uint8_t id;                    // this is the node if
  uint8_t group;                 // the group
} RFM12Var;

extern RFM12Var rfm12;

/**
 * This is the buffer where we write the byte to transmit or the byte which we received
 */
extern volatile uint8_t rfm12Buffer[];
/**
 * Current index within the buffer
 */
extern volatile uint8_t rfm12Index;
/**
 * the status of the finite state machine
 */
extern volatile uint8_t rfm12State;

/**
 * initialize the RFM12 chip, 
 * the data structure for the RFM12. It also initialize the spi and attaches the interrupt handler.
 * @param id the node id 
 * @param band the band typical RF12_868MHZ in europe
 * @group the group id 0..255
 *
 * After this call the RFM12 chip sleeps
 */
void rfm12_setup( uint8_t id, uint8_t band, uint8_t group );

/**
 * Start to listen: now we are able to receive a message
 */
void rfm12_listen();

/**
 * true, if we have a message with a valid crc.
 */
boolean rfm12_didReceive();

/**
 * if rfm12_didReceive == true then process the message in rfm12Buffer
 * rfm12Buffer[ 0 ] : header (node id)
 * rfm12Buffer[ 1 ] : length of message 
 * rfm12Buffer[ 2 ] : here begins the data
 *
 * For the next message call rfm12_didProcess to reset the buffer and 
 * set the receiver on.
 */
void rfm12_didProcess();

/**
 * checks if we are able to send. If the result is true then we have to send the message
 * because the function will change the state. Therefore send the message otherwise the function will return false forever.
 */
boolean rfm12_canSend();

/**
 * Sends a message
 * @param ptr  Pointer to the byte array
 * @param len  length in bytes
 */
void rfm12_send( const void* ptr, uint8_t len );

/**
 * put the RFM12 to sleep and save some power
 */
void rfm12_sleep();

/**
 * wakes up the RFM12
 */
void rfm12_wakeup();

