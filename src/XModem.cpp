#include <FS.h>
#include <XModem.h>
#include "txmod_debug.h"

// Number of seconds until giving up hope of receiving sync packets from
// host.
#define SYNC_TIMEOUT 30
// Number of times we try to send a packet to the host until we give up
// sending..
#define MAX_RETRY    30

/* Initialize XModem session */
XModem::XModem(Stream *port, char mode)
{
  packetNo = 1;
  crcBuf = 0;
  checksumBuf = 0;
  filepos = 0;
  packetLen = 128; // Default number of payload bytes
  if (mode == ModeYModem)
  {
    this->mode = ModeYModem;
  } else
  {
    this->mode = ModeXModem;
  }
  this->port = port;
}

/* Send out a byte of payload data,
 * includes checksumming
 */
void XModem::outputByte(unsigned char inChar)
{
  char j;
  checksumBuf += inChar;

  crcBuf = crcBuf ^ (int) inChar << 8;
  j = 8;
  do
  {
      if (crcBuf & 0x8000)
    crcBuf = crcBuf << 1 ^ 0x1021;
      else
    crcBuf = crcBuf << 1;
  } while(--j);

  port->write(inChar);
}

/* Wait for either C or NACK as a sync packet.
 * Determines protocol details, like block size
 * and checksum algorithm.
 */
char XModem::sync(void)
{

    debug_serial_println("::sync" );
  char tryNo;
  char inChar;
  // Wait a second every time until timeout.
  // The magic char expected from the host is 'C' or
  // NAK
  port->setTimeout(1000);
  tryNo = 0;
  do
  {
    port->readBytes(&inChar, 1);
    tryNo++;
    // When timed out, leave immediately
    if (tryNo == SYNC_TIMEOUT)
      return(-1);
    debug_serial_println("::sync..." );
  } while ((inChar != 'C') && (inChar != NAK));
  // Determine which checksum algorithm to use
  // this routine also determines the packet length

    debug_serial_println("::sync-cont" );
  this->packetLen = 128;
  if (inChar == NAK)
    this->oldChecksum=1;
  else
  { // This is assumed to be 'C'
    this->oldChecksum=0;
    // Check if there is a K after the C, that means we can do 1024 byte blocks
    // Give sender a short time to send the K
    port->setTimeout(100);
    tryNo=0;
    port->readBytes(&tryNo, 1);
    if (tryNo=='K')
    {
      this->packetLen = 1024;
    }
    // Reset the timeout to one second
    port->setTimeout(1000);
  }
    
  return(0);
}

/** Wait for the remote to acknowledge or cancel.
  * Returns the received char if no timeout occured or
  * a CAN was received. In this cases, it returns -1.
  **/
char XModem::waitACK(void)
{
    //debug_serial_println("::waitACK" );
  char i, inChar;
  i = 0;
  do
  {
    port->readBytes(&inChar, 1);
    i++;
    if (i>200)
      return(-1);
    if (inChar == CAN)
      return(-1);
  } while ((inChar != NAK) && (inChar != ACK) && (inChar != 'C'));
  
    #define LED 2
    digitalWrite(LED,!digitalRead(LED));  // toggle LED for user feedback
    debug_serial_print("." ); //feedback over serial too
    
  return(inChar);
}


int XModem::sendFile(File dataFile)
{

debug_serial_println("sendFile....." ); 

  unsigned char finished=0;
  char inChar = 0;
  unsigned int i;
  unsigned char tryNo;

  // Rewind data file before sending the file..
  dataFile.seek(0);
  
  if (this->sync()!=0) { 
    debug_serial_println("::sync failed" );
    digitalWrite(LED,HIGH);  // leave  LED OFF at end of upload if failed
    return -1;
  }

  oldChecksum =  (inChar == NAK);

  while (!finished)
  {
    filepos = dataFile.position();

       // debug_serial_println("\tXMODEM progress -->" ); 

    // Sending a packet will be retried
    tryNo = 0;
    do
    {
      // Seek to start of current data block, 
      // will advance through the file as
      // block will be acked..
      dataFile.seek(filepos);

      // Reset checksum stuff
      checksumBuf = 0x00;
      crcBuf = 0x00; 

      // Try to send packet, so header first
      if (packetLen == 128)
        port->write(SOH);
      else
        port->write(STX);

      port->write(packetNo);
      port->write(~packetNo);
      for (i = 0; i<packetLen; i++)
      {
        inChar = dataFile.read();
        this->outputByte(inChar);
        finished = !dataFile.available();
        // Pad file with zeroes
        if (finished)
          inChar = 0x00;

      }
      // Send out checksum, either CRC-16 CCITT or
      // classical inverse of sum of bytes. 
      // Depending on how the received introduced himself
      if (oldChecksum)
        port->write((char)255-checksumBuf);
      else
      {
        port->write((char) (crcBuf >>8));
        port->write((char) (crcBuf & 0xFF));
      }

      inChar = waitACK();
      tryNo++;
      if (tryNo > MAX_RETRY) {
        debug_serial_println("Number of tries exceeded");
        digitalWrite(LED,HIGH);  // leave  LED OFF at end of upload if failed
        return -1;
      }
    } while (inChar != ACK);
    
    packetNo++;
  }
  // Send EOT and wait for ACK
  tryNo = 0;
  do
  {
    port->write(EOT);
    inChar = waitACK();
    tryNo++;
    // When timed out, leave immediately
    if (tryNo == SYNC_TIMEOUT) {
      debug_serial_println("Synchronisation timed out");
      digitalWrite(LED,HIGH);  // leave  LED OFF at end of upload if failed
      return -1;
    }
  } while (inChar != ACK);

  // When we get here everything was successful.
  digitalWrite(LED,LOW);  // leave  LED ON at end of upload if successful
  return 1;

}
