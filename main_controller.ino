/*
 BM 2015 Chuckie Pinball by RCI and TonyOxide
 This code is in the public domain.
 Version 0.03
 
 Changelog
 v 0.03 Adding CD4067 Driver - main program flow
 */

#define TRUE  1
#define FALSE 0

//State Machine
#define NO_PLAYER_DETECTED 0
#define PLAYER_DETECTED 1
#define COUNTING_PLAYER_FINGERS 2
#define ALIGN_PLAYER_TO_MIRROR 3
#define WAIT_FOR_PLAYER_TO_LEAVE 4

//Pinout
#define PIN_MIRROR_LIGHTS 6
#define PIN_SOLENOID 4

//Distance
#define DISTANCE_SENSOR_FLOOR 60
#define DISTANCE_SENSOR_CEILING 200

#define FINGER_DEBOUCE_LENGTH 300 //Abitrary - counting in loop cycles right now

#define PLAYER_DISTANCE_TIMEOUT_LENGTH 300

// Turn debugging on or off...
// Don't even try to use #if syntax, the IDE breaks if you do
// Bitmask 
// 1 = LEDs
// 2 = PIRs
// 4 = FINGERs
// 8 = MachineState

char TDEBUG = 0;

unsigned char pin_led_test = 13;
unsigned char pin_sen_pir_test = 7;
unsigned int  sen_pir_test = 0; // PIR sense value

// Variables you might want to change
unsigned char pin_led_red = 9;      // Red LED pin
unsigned char pin_led_green = 10;   // Green LED pin
unsigned char pin_led_uv = 11;      // UV LED pin
unsigned char pin_face_sen = 14;    // Face sensor pin
unsigned long delay_red_green_min = 5000; // minimum time red/green is on
unsigned long delay_red_green_max = 10000; // maximum time red/green is on
unsigned long delay_fade = 2000;    // Fade between red/green/uv
float delay_led_cross = 600; // Minimum amount of time to increment 1 crossfade value in ms
unsigned long random_uv_led_pool = 3000; // 1 out of every xx seconds, light the UV LED instead
unsigned char uv_led_on = 850; // Time to flash UV LED
unsigned char uv_led_on_buffer = 0; // Time to turn off all other LEDS before/after UV LED
unsigned long uv_led_on_time = 0; // Time that the UV LED was turned on
unsigned int sen_light_min = 900;   // Minimum value for light sensor
unsigned char loop_delay = 2;      // Default loop delay in ms

// Variables you probably don't want to change
unsigned long ctr_time; // Time since boot in ms
unsigned long ctr_time_prv; // Time of the start of the previous loop
unsigned int time_loop_delay = 0; // Delay of last loop run
unsigned char pin_led_cross_on = pin_led_red;  // Crossfade on/off value
unsigned char pin_led_cross_off = pin_led_green; // Crossfade on/off value
float led_level_change = 0; // Value to change for LED
unsigned long led_level_change_prv = 0; // Last time the LED level changed
unsigned long led_level_change_start = 0; // Fade start time

unsigned char pin_sen_light = A5;  // Light sensor pin
int led_cross_on = 0; // Crossfade level for LED turning on
int led_cross_off = 255; // Crossfade level for LED turning off

unsigned int sen_light = 0; // Light sensor absolute value
unsigned char light_on_brightness = 255; // Max brightness of the LEDs by default (0-255))
unsigned char led_bright_red = 0; // Normalized value of LED brightness
unsigned char led_bright_green = 0; // Normalized value of LED brightness
unsigned char led_bright_uv = 0; // Normalized value of LED brightness

unsigned long delay_red_green_random = 0;  // Init random delay interval

unsigned long led_on_time = 0; // Time to turn the LED on
unsigned char led_bright = 0; // Brightness of the LED
unsigned char pin_green = 0; // Green LED pin

// Counter init
unsigned int ctr_red = 1;   // Number of red activations (
unsigned int ctr_green = 0; // Number of green activations
unsigned int ctr_uv = 0;    // Number of UV activations
unsigned int ctr_loop = 0;  // Number of loops

unsigned long time_led_start = 0;  // Time LED went on

//CD4067B variables
unsigned char muxInputPin = A5;    // mux input to arduino
unsigned char muxPinD = A4;     //mux Address line
unsigned char muxPinC = A3;     //mux Address line
unsigned char muxPinB = A2;     //mux Address line
unsigned char muxPinA = A1;     //mux Address line
unsigned char muxResult = 0;  //Current mux line result
unsigned char totalFingersCounted = 0;
unsigned char lastFingerCount = 0;


//TCB380
unsigned char soundFileActive = FALSE;
unsigned char tcb380Active = 3;    //sound file output

//User State Variables
unsigned char machineState = NO_PLAYER_DETECTED; //Check define tablefor states
unsigned char userInProximity = 0;
unsigned int  userDistance = 0;
unsigned int fingerDebounceCount = 0;
unsigned int playerDetectedTimeout = 0;

// This function looks for someone standing near the exhibit
int pir_check(int pin = 0) {
  int val = 0;
  val = digitalRead(pin);
  if (val == 0) {
    digitalWrite(pin_led_test,LOW);
  } else {
    digitalWrite(pin_led_test,HIGH);
  }
  if (TDEBUG & 2) {
      Serial.print("pir=");
      Serial.println(val);
      delay(10);
  }
}


void setup() {
  randomSeed(analogRead(0));
  // Turn on Green LED
  delay_red_green_random = random(delay_red_green_min, delay_red_green_max);
  pin_led_cross_off = pin_led_red;
  ctr_red = 1;
  time_led_start = millis() - delay_red_green_random - 1; // Turn an LED on right away
  
  // mux outputs
  pinMode(muxPinA, OUTPUT); 
  pinMode(muxPinB, OUTPUT);
  pinMode(muxPinC, OUTPUT);
  pinMode(muxPinD, OUTPUT);
  //mux input is analog
  pinMode(muxInputPin, INPUT);
  
  //Extra hardware
  pinMode(PIN_MIRROR_LIGHTS, OUTPUT);
  pinMode(PIN_SOLENOID, OUTPUT);

  //TCB380 Initilization
  Serial.begin(4800);
  //Serial.write(0xE0); //Set Volume 0xC8-E7
  delay(50);
  pinMode(tcb380Active, INPUT);
  
  led_test();
  lightBoxTest();
  solenoidTest();
  
  if (TDEBUG == 1) {
    pinMode(pin_sen_pir_test, INPUT);
    pinMode(pin_led_test, OUTPUT);
  }
}

void loop() {
	// Housekeeping
  ctr_time_prv = ctr_time; // Set the previous value of the timer
  ctr_time = millis();
	time_loop_delay = ctr_time - ctr_time_prv; // Delay since the last loop
  ctr_loop++;

  // See what we should do with the illumination LEDs
  panel_checkFade();	

  // See what is happening with the PIR sensors
  sen_pir_test = pir_check(pin_sen_pir_test);
  
  //Scan the mux to count fingers
  lastFingerCount = totalFingersCounted;
  totalFingersCounted = scanFingerSensors();

  userInProximity = readFrontPIRSensor(); //userInProximity holds the PIR Sensor
  userDistance = readDistanceSensor();  //userDistance is the distance!
  
  //Solenoid test
  /*if(totalFingersCounted == 3){
    digitalWrite(PIN_SOLENOID, HIGH);
  }else{
    digitalWrite(PIN_SOLENOID, LOW);
  }
  
  //Lightbox test
  if(totalFingersCounted == 2){
    digitalWrite(PIN_MIRROR_LIGHTS, HIGH);
  }else{
    digitalWrite(PIN_MIRROR_LIGHTS, LOW);
  }*/
  
  /*if(totalFingersCounted != lastFingerCount){
    Serial.write(totalFingersCounted-1);
  }*/
  
  //No user detected - Bark for attention  
  
  //Finger count routine begins - overrides PIR   
  if((totalFingersCounted > 0) &&
     (machineState == NO_PLAYER_DETECTED) ||
     (machineState == PLAYER_DETECTED))
  {
    machineState = COUNTING_PLAYER_FINGERS;
  }
  
  //Debounce finger input sensors
  //until the number of fingers is stable
  if((machineState == COUNTING_PLAYER_FINGERS) &&
      (totalFingersCounted == lastFingerCount))
  {
    fingerDebounceCount++;
  }
  
  //Finger debounce complete - Speak the count
  if(fingerDebounceCount > FINGER_DEBOUCE_LENGTH){
    Serial.write(totalFingersCounted-1);
    //soundFileActive = digitalRead(tcb380Active);
    //Serial.println(soundFileActive);    
    fingerDebounceCount = 0;
    //machineState = NO_PLAYER_DETECTED;
    //Fire solenoid
  }
    
  //If the front PIR detects movement OR the distance sensor is below threshold
  if((machineState == NO_PLAYER_DETECTED) && userInProximity){
      machineState = PLAYER_DETECTED;
  }
  
  
  //Player in the area - Bark at them to come over
  if(machineState == PLAYER_DETECTED){
      playerDetectedTimeout = PLAYER_DISTANCE_TIMEOUT_LENGTH;
  }
  
  //Count until timeout or user overrides by putting fingers in! 
  if(playerDetectedTimeout > 0){
      playerDetectedTimeout--; //Count down until timeout is complete
      if(playerDetectedTimeout == 0){ //Timeout is complete 
        machineState = NO_PLAYER_DETECTED;
      }
  }
  
  /*if((userInProximity == TRUE) && !soundFileActive){
    //Tell user to get closer and put fingers in
    Serial.write(1);
  }*/
  
  //Add timer to call the user a spoilsport if they won't come
  
  //Begin Magic Mirror routine
    //Guide user to optimal distance
    //If above threshold - speak "closer"
    //If below threshold - speak "farther away" 
    //Fire Mirror lighting
    //Laugh
    //Instruct user to find their true selves
  
  //Timer for user to leave
  //Tell them to scram on timeout
  //Back to  outer loop
  
  
  //Serial.println(userDistance);
  if(TDEBUG & 8){
    Serial.println(machineState);
  }
  
  //Check Rear PIR - yell at hooligans behind the machine
  
  digitalWrite(PIN_SOLENOID, LOW); //Safety - Always deactivate the solenoid
  
  // Delay at the end of the loop (optional)
  delay(loop_delay);
}

void panel_checkFade() { // See if we should fade to another LED for the panel
	led_on_time = ctr_time - time_led_start;
	// See if we need to flash the UV LED
	if (random(random_uv_led_pool) == 1 || uv_led_on_time > 0) { // Randomly display UV LED
		if (uv_led_on_time > 0) { // UV is currently on
			// See if we should turn it off
			if (ctr_time - uv_led_on_time > uv_led_on) { // on long enough, turn it off
				analogWrite(pin_led_uv, 0);
				uv_led_on_time = 0;
			}
		} else {
		 // Turn on UV LED
			uv_led_on_time = ctr_time;
			analogWrite(pin_led_uv, led_bright_uv);
			ctr_uv++;
		}		 
	}
  
  if ( led_on_time > delay_red_green_random) { // Time to switch to the alternate LED
    delay_red_green_random = random(delay_red_green_min, delay_red_green_max);
    time_led_start = millis();
		
    if ( ctr_red == 0 && ctr_green > 1000) { // Counter rolled over, reset counters
      ctr_green = 0;
      ctr_red = 1;
    }
		
    pin_led_cross_off = pin_led_cross_on; // Previous pin that was on
		// Set relative brightness for the LEDs
		led_bright_red = light_on_brightness * 1;
		led_bright_green = light_on_brightness * 1;
	  led_bright_uv = light_on_brightness * 1;
		
		if ( ctr_red > ctr_green) {
			pin_led_cross_on = pin_led_green;
			led_cross_off = led_bright; // Set cross-fade max value for LED turning off
			led_bright = led_bright_green;
			ctr_green++;
		} else {
			pin_led_cross_on = pin_led_red;
			led_cross_off = led_bright; // Set cross-fade max value for LED turning off
			led_bright = led_bright_red;
			ctr_red++;
		}
		
    led_cross_on = 0;  // Set cross-fade value to 0 for the LED turning on
    led_level_change_start = millis();
		if (TDEBUG & 1) {
			Serial.print("led_bright= ");
			Serial.print(led_bright);
			Serial.print("\tled_red= ");
			Serial.print(led_bright_red);
			Serial.print("\tled_green= ");
			Serial.println(led_bright_green);
		}

  } else { // Not turning a new LED on, see if we need to cross-fade
    if (led_cross_on < led_bright || led_cross_off != 0) { // LED to turn on is not yet at full brightness, increase by 1 level
      // See how many levels to change the LED
			
			ctr_time = millis();
			// Delay between last change and now
      led_level_change = (ctr_time - led_level_change_start) / delay_led_cross;
      if (led_level_change > 1) { // Too long, turn them off/on
        led_level_change = 1;
      }
			
      led_cross_on = led_level_change * led_bright;
      led_cross_off = led_bright - (led_level_change * led_bright);

      if (led_cross_on >= led_bright || led_cross_off < 0) {  // Values too big or too small, just punt
        led_cross_on = led_bright;
        led_cross_off = 0;
      }
			
      analogWrite(pin_led_cross_on, led_cross_on);
      analogWrite(pin_led_cross_off, led_cross_off);
      // print the results to the serial monitor:
			if (TDEBUG & 1) {
				Serial.print("t=");
				Serial.print(ctr_time);
				Serial.print("\tst=");
				Serial.print(led_level_change_start);
				Serial.print("\tled_bright=");
				Serial.print(led_bright);
				Serial.print("\tledon=");
				Serial.print(led_cross_on);
				Serial.print("\tledoff=");
				Serial.print(led_cross_off);
				Serial.print("\tledlevel=");
				Serial.print(led_level_change);
				Serial.print("\t pin=");
				Serial.print(pin_led_cross_on);
				Serial.print("\t chg=");
				Serial.println((ctr_time - led_level_change_start) / delay_led_cross);
				delay(20);
			}
    }
  }
}

//Reads finger inputs from mux into array
unsigned char scanFingerSensors(void){

  unsigned char tempFingerCount = 0; //current finger count for return
  unsigned char tempMuxResult = 0;
  
  //Finger sensors are on first ten mux inputs
  for(unsigned char i = 0; i < 10; i++){
    tempMuxResult = readMuxPin(i);
    if(TDEBUG == 3){
      Serial.print(tempMuxResult);
    }
    tempFingerCount = tempFingerCount + tempMuxResult;
    tempMuxResult = 0;
  }
  if(TDEBUG == 3){
    Serial.println("");
  }
  
  return tempFingerCount;
}

//Check the Front PIR Sensor
unsigned char readFrontPIRSensor(){
  unsigned char result = 0;
  
  result = readMuxPin(10);
  if(TDEBUG == 4){
    Serial.print("PIR = ");
    Serial.println(result);
  }
  return result;
  
}

//Check the distance sensor
unsigned char readDistanceSensor(){
  unsigned char result = 0;
  
  result = readMuxPin(12);
  
  if(TDEBUG == 4){
    Serial.print("Distance = ");
    Serial.println(result);
  }
  
  return result;
}

//Pass mux input to read
//Returns digital result except for distance sensor
unsigned int readMuxPin(unsigned char muxPin){

  unsigned int result;

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

    //Front PIR
    case 10:
      //set address lines
      digitalWrite(muxPinA, 0);
      digitalWrite(muxPinB, 1);
      digitalWrite(muxPinC, 0);
      digitalWrite(muxPinD, 1);
      result = digitalRead(muxInputPin);
      break;
    
    //Rear PIR
    case 11: 
      //set address lines
      digitalWrite(muxPinA, 1);
      digitalWrite(muxPinB, 1);
      digitalWrite(muxPinC, 0);
      digitalWrite(muxPinD, 1);
      result = digitalRead(muxInputPin);
      break;    
      
    case 12: //Distance Sensor
      //set address lines
      digitalWrite(muxPinA, 0);
      digitalWrite(muxPinB, 0);
      digitalWrite(muxPinC, 1);
      digitalWrite(muxPinD, 1);      
      delay(10);
      result = analogRead(muxInputPin);
      break;      

    default:
      break;
  }

  return result;
}

// Test functions
void led_test() { // Startup test for LEDs
	analogWrite(pin_led_red, 255);
  analogWrite(pin_led_green, 255);
  analogWrite(pin_led_uv, 255);
	delay(5000);
	analogWrite(pin_led_red, 0);
  analogWrite(pin_led_green, 0);
  analogWrite(pin_led_uv, 0);
	
}

void lightBoxTest(){
  digitalWrite(PIN_MIRROR_LIGHTS, HIGH);
  delay(1000);
  digitalWrite(PIN_MIRROR_LIGHTS, LOW);
}

void solenoidTest(){
  digitalWrite(PIN_SOLENOID, HIGH);
  delay(500);
  digitalWrite(PIN_SOLENOID, LOW);
}
