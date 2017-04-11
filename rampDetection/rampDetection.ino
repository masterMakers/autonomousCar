//Ramp Detection
bool rampDetected = false;
int reflSensorPin = A10;
double sum;
int counter = 0;
int n = 3;
double reflValues[3];
double reflValueSmooth = 0;
double rampThreshold;


void setup() 
{
  //start serial communication
  Serial.begin(115200);
  
  //initialize pins
  pinMode(reflSensorPin, INPUT);
  
  //init reflectance value array
  for(int i=0; i<n; i++)
  {
    reflValues[i] = 0.0;  
  }
  
  //calibrate ramp detection threshold
   rampThreshold = analogRead(reflSensorPin) * 1.1;
   Serial.print(rampThreshold);
}

void loop() 
{    
    reflValues[counter%n] = analogRead(reflSensorPin);
    
    //filter out obviouse noise
    if (reflValues[counter%n]>1000)
    {
      reflValues[counter%n] = reflValues[(counter-1)%n];
    }
    
    //rolling average filter for smoothing
    sum = 0.0;
    for(int i=0; i<n; i++)
    {
      sum += reflValues[i];
    }
    reflValueSmooth = sum / n;
    
    //check for ramp
    if(reflValueSmooth > rampThreshold)
    {
      rampDetected = 1; 
      Serial.print("Ramp Detected!");
    }
    else
    {
      rampDetected = 0; 
    }
    
    //Serial.print(reflValueSmooth); 
    Serial.println();
   
    counter++;
    delay(20);
}
