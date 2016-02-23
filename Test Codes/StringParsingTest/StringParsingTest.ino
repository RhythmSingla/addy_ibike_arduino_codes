#include <String.h>

String inputString = "22.87451695,88.65114785,16$";
boolean strcmplete = false;

float LATI, LONGI;
int THETA;

String lati = "", longi = "", theta = "";

void parse()
{
  int len = inputString.length();
  int i=0, j=0, k=0;
  
  for(i=0; inputString[i] != ','; i++)
  {
    lati += inputString[i];  
  }
  i++;
  lati[i] = '\0';
  
  for(j=0; inputString[i] != ','; i++,j++)
  {
    longi += inputString[i];  
  }
  i++;
  j++;
  longi[j] = '\0';
  
  for(; inputString[i] != '$'; i++,k++)
  {
     theta += inputString[i]; 
  }
  k++;
  theta[k] = '\0';
  
  LATI = lati.toFloat();
  LONGI = longi.toFloat();
  THETA = theta.toInt();
}

void setup()
{
  Serial.begin(115200);
  lati.reserve(12);
  longi.reserve(12);
  parse();
  delay(500);
  
  Serial.println(inputString); 
  Serial.println(lati); 
  Serial.println(longi);
  Serial.println(theta); 
  Serial.println();
  Serial.println(LATI); 
  Serial.println(LONGI);
  Serial.println(THETA);  
}

void loop()
{
  
}
