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

h3. Sending a message

First initialize the RFM12 data structure and the RFM12 itself. In europe use <code>RF12_868MHZ</code>. The
example use node id 2 and group 200. After that check if sending is possible by calling <code>rfm12_canSend()<code>.
If it returns true then you have to send your package with <code>rfm12_send()</code>. 

<pre><code>

#include "rfm12.h"

typedef struct {
  uint16_t temp;
  uint8_t counter;
} Payload;

Payload            payload;    

void setup() {   

  rfm12_setup( 2, RF12_868MHZ, 200 );
}

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
