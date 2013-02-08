RFM12
=====

An Arduino implementation for using the RFM12 transceiver from Hoperf. This implementation uses
the same message format as the JeeNodes:

* header, which contains the node id
* length, the length of the message
* data, the data of the message
* crc, the crc in little-endian format

The code uses a finite state machine which organise the operation mode of the RFM12. You can
reveive, send messages or power down the RFM12. 

# Sending a message

First initialize the RFM12 data structure and the RFM12 itself. In europe use <code>RF12_868MHZ</code>. The
example use node id 2 and group 200. After that check if sending is possible by calling <code>rfm12_canSend()</code>.
If it returns true then you have to send your package with <code>rfm12_send()</code>. 

<pre><code>
#include "rfm12.h"

typedef struct {
  uint16_t temp;
  uint8_t counter;
} Payload;

Payload payload;    

void setup() { rfm12_setup( 2, RF12_868MHZ, 200 ); }

void loop() {   
  if( rfm12_canSend() ) {
    payload.temp= 20.10;
    payload.counter= 1;
    rfm12_send( &payload, sizeof payload );
  }
  ...
}
</code>
</pre>

# Receiving a message

To receive the message that we sent before. We initializes the RFM12 with node id 1 and group 200.
By calling <code>rf12mListen()</code> we turn on the reveiver, if it is possible. Call <code>idReceived()</code>
to check if the RFM12 did received a message. The message will be written the buffer <code>rf12Buffer</code> which
is also used for sending. After receiving the message you should process the message in the buffer. While
processing the buffer won't be overwritten by the next message. By calling <code>didProcessed()</code> the state
of the fsm (finite state machine) will be change to be idle. Now you can receive the next message or 
answer by sending a message.

<pre><code>
#include "rf12m.h"

typedef struct {
  uint16_t temp;
  uint8_t counter;
} Payload;

void setup () { rf12m_init( 1, RF12_868MHZ, 200 ); }

void loop () {
  
  rf12mListen();  
  
   if (didReceived() ) {
     Payload* p= (Payload*)&rf12Buffer[ 2 ];        
     Serial.print( p->counter );
     Serial.print( ". Temperature:" );
     Serial.println( (float)p->temp / 100.0 );
     didProcessed();
    }      
}
</code>
</pre>

# Just code

This is not a library. It is just some code that you can grab and modify it for your needs.
Out of the box it supports Attiny84 and ATMega328. For using other microcontroller just change the
pins and interrupt handlers. If you use ATMega328 you can use the debug mode <code>#define RFM12_DEBUG 1</code> that puts some strings on the serial usb. 

