
volatile int incomingYaw = 0;

void setup() {
  // initialize serial:
  Serial1.begin(115200);
  Serial.begin(115200);  
}

void loop() 
{
  
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
int serialEvent1() {
  while(Serial1.available())
  {
    // get the new byte:
    incomingYaw = (int)Serial1.read(); 
    if(incomingYaw >= 128)
    {
      incomingYaw -= 255;
    }
    Serial.print("Yaw: "); Serial.println(incomingYaw);
  }
}


