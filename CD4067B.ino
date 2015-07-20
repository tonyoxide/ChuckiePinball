/* CD4067B Driver

 
 
 
 */

#define NUMBER_OF_FINGER_SENSORS 10

unsigned char muxInputPin = A5;    // mux input to arduino
unsigned char muxPinD = A4;     //mux Address line
unsigned char muxPinC = A3;     //mux Address line
unsigned char muxPinB = A2;     //mux Address line
unsigned char muxPinA = A1;     //mux Address line

unsigned char muxResult = 0;  //Current mux line result

unsigned char totalFingersCounted = 0;

void setup() {

  // mux outputs
  pinMode(muxPinA, OUTPUT); 
  pinMode(muxPinB, OUTPUT);
  pinMode(muxPinC, OUTPUT);
  pinMode(muxPinD, OUTPUT);
  //mux input is analog
  pinMode(muxInputPin, INPUT);

  Serial.begin(9600);
}

void loop() {

  // stop the program for for <sensorValue> milliseconds:
  delay(1000);
  //Serial.println(muxResult);  

  scanFingerSensors();
  //muxResult = readMuxPin(1);
  //delay(1000);              
  //Serial.println(muxResult);
}

//Reads finger inputs from mux into array
unsigned char scanFingerSensors(void){

  unsigned char tempFingerCount = 0; //current finger count for return
  unsigned char tempMuxResult = 0;
  
  for(unsigned char i = 0; i < NUMBER_OF_FINGER_SENSORS; i++){
    tempMuxResult = readMuxPin(i);
    Serial.print(tempMuxResult);
    tempFingerCount = tempFingerCount + tempMuxResult;
    tempMuxResult = 0;
  }
  Serial.println("");
  
  return tempFingerCount;
}

//Pass mux input to read
//Returns digital result except for distance sensor
unsigned char readMuxPin(unsigned char muxPin){

  unsigned char result;

  switch(muxPin){
  
    case 0:
      //set address lines
      digitalWrite(muxPinA, 0);
      digitalWrite(muxPinB, 0);
      digitalWrite(muxPinC, 0);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;
  
    case 1:
      //set address lines
      digitalWrite(muxPinA, 1);
      digitalWrite(muxPinB, 0);
      digitalWrite(muxPinC, 0);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;
  
    case 2:
      //set address lines
      digitalWrite(muxPinA, 0);
      digitalWrite(muxPinB, 1);
      digitalWrite(muxPinC, 0);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;

    case 3:
      //set address lines
      digitalWrite(muxPinA, 1);
      digitalWrite(muxPinB, 1);
      digitalWrite(muxPinC, 0);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;
  
    case 4:
      //set address lines
      digitalWrite(muxPinA, 0);
      digitalWrite(muxPinB, 0);
      digitalWrite(muxPinC, 1);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;

    case 5:
      //set address lines
      digitalWrite(muxPinA, 1);
      digitalWrite(muxPinB, 0);
      digitalWrite(muxPinC, 1);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;
    
    case 6:
      //set address lines
      digitalWrite(muxPinA, 0);
      digitalWrite(muxPinB, 1);
      digitalWrite(muxPinC, 1);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;
      
    case 7:
      //set address lines
      digitalWrite(muxPinA, 1);
      digitalWrite(muxPinB, 1);
      digitalWrite(muxPinC, 1);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;

    case 8:
      //set address lines
      digitalWrite(muxPinA, 0);
      digitalWrite(muxPinB, 0);
      digitalWrite(muxPinC, 0);
      digitalWrite(muxPinD, 1);
      result = digitalRead(muxInputPin);
      break;
  
    case 9:
      //set address lines
      digitalWrite(muxPinA, 1);
      digitalWrite(muxPinB, 0);
      digitalWrite(muxPinC, 0);
      digitalWrite(muxPinD, 1);
      result = digitalRead(muxInputPin);
      break;

    case 10:
      //set address lines
      digitalWrite(muxPinA, 0);
      digitalWrite(muxPinB, 1);
      digitalWrite(muxPinC, 1);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;
      
    case 11: //Front PIR
      //set address lines
      digitalWrite(muxPinA, 1);
      digitalWrite(muxPinB, 1);
      digitalWrite(muxPinC, 1);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;    
      
    case 12: //Rear PIR
      //set address lines
      digitalWrite(muxPinA, 1);
      digitalWrite(muxPinB, 1);
      digitalWrite(muxPinC, 1);
      digitalWrite(muxPinD, 0);
      result = digitalRead(muxInputPin);
      break;    

    case 13: //Distance Sensor
      //set address lines
      digitalWrite(muxPinA, 1);
      digitalWrite(muxPinB, 1);
      digitalWrite(muxPinC, 1);
      digitalWrite(muxPinD, 0);
      result = analogRead(muxInputPin);
      break;    

    default:
      break;
  }

  return result;
}
