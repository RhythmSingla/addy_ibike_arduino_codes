int cnt = 0;
volatile int inData=0;
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);  
}

void loop()
{
   if(cnt == 255)
   {
     cnt = 0;  
   }
   cnt++;
   //Serial.println(cnt);
   Serial1.print(cnt);
   Serial1.println("\t I am Tony Stark.");
   delay(5000);
}

int serialEvent1() {
  while(Serial1.available())
  {
    // get the new byte:
    inData = (int)Serial1.read(); 
    
    Serial.print("inData :"); Serial.println(inData);
  }
}

