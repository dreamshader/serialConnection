#include "serialConnection.h"

serialConnection *pXConnection;
char dataBuffer[20];
 
void setup() {
  // put your setup code here, to run once:
  if( (pXConnection = new serialConnection( &Serial, 57600 )) != NULL )
  {
    pXConnection->ser_open();
  }

  sprintf(dataBuffer, "Hello World!\n");
}


void loop() {
  // put your main code here, to run repeatedly:
  if( pXConnection != NULL )
  {
    pXConnection->ser_write(dataBuffer, strlen(dataBuffer));
  }
  delay(2000);
}

