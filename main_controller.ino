/*
 BM 2015 Chuckie Pinball by RCI and TonyOxide
 This code is in the public domain.
 Version 0.03
 
 Changelog
 v 0.03 Adding CD4067 Driver - main program flow
 */

// Defines for all of the sound files
//1,01,One
#define SND_1 1
//2,02,Two
#define SND_2 2
//3,03,Three
#define SND_3 3
//4,04,Four
#define SND_4 4
//5,05,Five
#define SND_5 5
//6,06,Six
#define SND_6 6
//7,07,Seven
#define SND_7 7
//8,08,Eight
#define SND_8 8
//9,09,Nine
#define SND_9 9
//10,0A,Ten
#define SND_10 10
//11,0B,Nine! Nine! Nine! Nine!
#define SND_NINE 11
//12,0C,Ha ha ha ha ha!
#define SND_HAHA 12
//13,0D,Ho Ho Ho Ho!
#define SND_HOHO 13
//14,0E,Gotcha!
#define SND_GOTCHA 14
//15,0F,Wooo Wooo Wooo!
#define SND_WOO 15
//16,10,Sacre Bleu!
#define SND_SACRE 16
//17,11,Bon Ben! (bone-baaa)  [OK well]
#define SND_BON 17
//18,12,Quelle Surprise! (Kelle sur preeze!) 
#define SND_QUELLE 18
//19,13,Look at THAT one!
#define SND_LOOKAT 19
//20,14,"Come on over, it's YOUR lucky day!"
#define SND_COMEOVER 20
//21,15,Wooo! Tu est Belle!
#define SND_WOO 21
//22,16,"Hey fabulous, come over here!"
#define SND_HEY 22
//23,17,Find out who you really are!
#define SND_FIND 23
//24,18,"Two faces have I, one is truth and one a lie!"
#define SND_TWOFACE 24
//25,19,I've been waiting for you.....
#define SND_WAITING 25
//26,1A,"Come over here, let me count your fingers!"
#define SND_COUNT 26
//27,1B,Put your fingers in the little holes.
#define SND_FINGERS 27
//28,1C,The holes on the sides!
#define SND_HOLES 28
//29,1D,You must be scared! (If hesitation)
#define SND_SCARED 29
//30,1E,"OK, done with counting! (If total counting time exceeds 30 seconds)"
#define SND_DONECOUNT 30
//31,1F,Now look into my magic mirror.
#define SND_LOOKMIRROR 31
//32,20,"Come a little closer, so your face fits in."
#define SND_COMECLOSER 32
//33,21,Not THAT close!
#define SND_TOOCLOSE 33
//34,22,Perfect. Hold it!
#define SND_HOLDIT 34
//35,23,You look great!
#define SND_LOOKGREAT 35
//36,24,Voo Voo Voo Voo!
#define SND_VOO 36
//37,25,All done with the mirror. (if they are still there 15 seconds later)
#define SND_DONEMIRROR 37
//38,26,Now come inside and find your TRUE identity! 
#define SND_COMEINSIDE 38
//39,27,"Now come in, see yourself as you see others!"
#define SND_COMEIN 39

// Create arrays for the sound file types
int asnd_laugh[] = {
  11,12,13,14};
int asnd_attract[] = {
  14,15,16,17,18,19.20,21,22,23,24,25};
int asnd_leave[] = {
  38,39};

#define TRUE  1
#define FALSE 0

//State Machine
#define NO_PLAYER_DETECTED 0
#define PLAYER_DETECTED 1
#define COUNTING_PLAYER_FINGERS 2
#define SOLENOID_ACTIVE 3
#define ALIGN_PLAYER_TO_MIRROR 4
#define PLAYER_ALIGNED_TO_MIRROR 5
#define WAIT_FOR_PLAYER_TO_LEAVE 6

//Pinout
#define PIN_MIRROR_LIGHTS 6
#define PIN_SOLENOID 4

//Distance
#define DISTANCE_SENSOR_TO_FAR 100
#define DISTANCE_SENSOR_TO_CLOSE 120
#define FINGER_DEBOUCE_LENGTH 250 //counting in loop cycles right now

#define PLAYER_DISTANCE_TIMEOUT_LENGTH 300 //counting in loop cycles right now


// Turn debugging on or off...
// Don't even try to use #if syntax, the IDE breaks if you do
// Bitmask 
// 1 = LEDs
// 2 = PIRs
// 4 = FINGERs
// 8 = MachineState
// 16 = MP3Commands

char TDEBUG = 0;

#define pin_led_test 13

// Variables you might want to change
#define pin_led_red 9      // Red LED pin
#define pin_led_green 10   // Green LED pin
#define pin_led_uv 11      // UV LED pin
#define pin_face_sen 14    // Face sensor pin
#define delay_red_green_min 5000 // minimum time red/green is on
#define delay_red_green_max 10000 // maximum time red/green is on
#define delay_fade 2000    // Fade between red/green/uv
float delay_led_cross = 600; // Minimum amount of time to increment 1 crossfade value in ms
#define random_uv_led_pool 3000 // 1 out of every xx seconds, light the UV LED instead
#define uv_led_on 850 // Time to flash UV LED
#define uv_led_on_buffer 0 // Time to turn off all other LEDS before/after UV LED
#define loop_delay 2      // Default loop delay in ms

// Variables you probably don't want to change
unsigned long uv_led_on_time = 0; // Time that the UV LED was turned on
unsigned long ctr_time = 0; // Time since boot in ms
unsigned long ctr_time_prv = 0; // Time of the start of the previous loop
unsigned int time_loop_delay = 0; // Delay of last loop run
unsigned char pin_led_cross_on = pin_led_red;  // Crossfade on/off value
unsigned char pin_led_cross_off = pin_led_green; // Crossfade on/off value
float led_level_change = 0; // Value to change for LED
unsigned long led_level_change_prv = 0; // Last time the LED level changed
unsigned long led_level_change_start = 0; // Fade start time

int led_cross_on = 0; // Crossfade level for LED turning on
int led_cross_off = 255; // Crossfade level for LED turning off

unsigned int  sen_light = 0; // Light sensor absolute value
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
#define muxInputPin A5    // mux input to arduino
#define muxPinD A4     //mux Address line
#define muxPinC A3     //mux Address line
#define muxPinB A2     //mux Address line
#define muxPinA A1     //mux Address line
unsigned char muxResult = 0;  //Current mux line result

//Finger Count variables and timers
unsigned char totalFingersCounted = 0;
unsigned char maxFingersCounted = 0;
unsigned char lastFingerCount = 0;
unsigned long fingerDebounceCounter = 0;
unsigned char countSpoken = FALSE;
#define FINGER_DEBOUNCE_LENGTH 50
unsigned long solenoidTimer = 0;

//TCB380
unsigned char soundFileActive = FALSE;
unsigned char lastSoundFileActiveState = FALSE;
unsigned char tcb380Active = 3;    //sound file output
unsigned int  timeLastFileCompleted = 1; //Initilize so we can play on bootup

//Mirror light variables
unsigned long mirrorLightTimer = 0;
unsigned char mirrorTimerStarted = FALSE;

//User State Variables
unsigned char machineState = NO_PLAYER_DETECTED; //Check define tablefor states
unsigned char userInProximity = 0;
unsigned int  userDistance = 0;
unsigned int fingerDebounceCount = 0;
unsigned int playerDetectedTimeout = 0;
unsigned char timeToLaugh = FALSE;
unsigned char laughPlayed = FALSE;
unsigned char distanceAchievedPlayed = FALSE;
unsigned char nowLookIntoMyMirrorPlayed = FALSE;
unsigned char selectRandomTaunt = 0;

// This function looks for someone standing near the exhibit
int pir_check(int pin = 0) {
  int val = 0;
  val = digitalRead(pin);
  if (val == 0) {
    digitalWrite(pin_led_test,LOW);
  } 
  else {
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
  Serial.write(0xE0); //Set Volume 0xC8-E7
  delay(50);
  pinMode(tcb380Active, INPUT_PULLUP);

  led_test();
  lightBoxTest();
  solenoidTest();

  if (TDEBUG == 1) {
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
	
  //Scan the mux to count fingers
  lastFingerCount = totalFingersCounted;

  totalFingersCounted = scanFingerSensors();
  if(lastFingerCount != totalFingersCounted){
    fingerDebounceCounter = ctr_time;
  }

  userInProximity = readFrontPIRSensor(); //userInProximity holds the PIR Sensor
  userDistance = readDistanceSensor();  //userDistance is the distance!

  //update the file active states
  lastSoundFileActiveState = soundFileActive;
  soundFileActive = !digitalRead(tcb380Active); //Active low from mp3 module while sound is play pin low

  //Update time when the busy line transitions to file inactive
  if(soundFileActive != lastSoundFileActiveState && !soundFileActive){
    timeLastFileCompleted = ctr_time;
  }

  //No user detected - Bark for attention  
  selectRandomTaunt = random(sizeof(asnd_attract));
  if(machineState == NO_PLAYER_DETECTED){
    playFile(selectRandomTaunt, 100);
  }

  //Finger count routine begins - overrides PIR   
  if((totalFingersCounted > 0) &&
    (machineState == NO_PLAYER_DETECTED) ||
    (machineState == PLAYER_DETECTED))
  {
    machineState = COUNTING_PLAYER_FINGERS;
  }

  //Debounce finger input sensors
  //until the number of fingers is stable
  if((machineState == COUNTING_PLAYER_FINGERS) && (totalFingersCounted < 10))
  {
    playFile(totalFingersCounted, 0);
    /*if(TDEBUG & 16){
     Serial.println((totalFingersCounted-1) + 48);
     }*/
  }  

  if((machineState == COUNTING_PLAYER_FINGERS) && (totalFingersCounted == 10) && !timeToLaugh){
    countSpoken = playFile(totalFingersCounted, 0);  //Max fingers achieved
    if(countSpoken){
      countSpoken = FALSE;
      solenoidTimer = ctr_time;
      timeToLaugh = TRUE;
      machineState = SOLENOID_ACTIVE;
    }
  }
  
  if(machineState == SOLENOID_ACTIVE){
    whackSolenoid();
  }
  
  if((machineState == SOLENOID_ACTIVE) || (machineState == ALIGN_PLAYER_TO_MIRROR) && timeToLaugh){
    laughPlayed = playFile(SND_NINE, 0); //Minus one HAHAHAHA
    if(laughPlayed){
      timeToLaugh = FALSE;
    }
  }


  //If the front PIR detects movement OR the distance sensor is below threshold
  if((machineState == NO_PLAYER_DETECTED) && userInProximity){
    machineState = PLAYER_DETECTED;
    playFile(SND_HEY, 0); //Minus one HAHAHAHA
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

  //Add timer to call the user a spoilsport if they won't come

  //Begin Magic Mirror routine
  if(machineState == ALIGN_PLAYER_TO_MIRROR && !nowLookIntoMyMirrorPlayed){
    nowLookIntoMyMirrorPlayed = playFile(31, 0);
  }
    
  if(machineState == ALIGN_PLAYER_TO_MIRROR){
    //Guide user to optimal distance
    
      //If above threshold - speak "closer"
      if(userDistance < DISTANCE_SENSOR_TO_FAR){
        playFile(0x20, 0);
      }
      else if(userDistance > DISTANCE_SENSOR_TO_CLOSE){
        playFile(0x21, 0);
      }
      else{ //User is the right distance
        distanceAchievedPlayed = playFile(0x22, 0);
        if(distanceAchievedPlayed){
          distanceAchievedPlayed = FALSE;
          nowLookIntoMyMirrorPlayed = FALSE;
          machineState = PLAYER_ALIGNED_TO_MIRROR;
          mirrorLightTimer = ctr_time;
        }
      }
    }
    
  //Instruct user to find their true selves
  
  if(machineState == PLAYER_ALIGNED_TO_MIRROR && !digitalRead(tcb380Active)){
    
    if(mirrorTimerStarted == FALSE){
      mirrorLightTimer = ctr_time;
      mirrorTimerStarted = TRUE;
    }
    flashMirrorLight();
  }

  if(machineState == WAIT_FOR_PLAYER_TO_LEAVE){

  }

  //Timer for user to leave
  //Tell them to scram on timeout
  //Back to  outer loop

  //Serial.println(userDistance);
  if(TDEBUG & 8){
    Serial.println(machineState);
  }

  //Check Rear PIR - yell at hooligans behind the machine

  //digitalWrite(PIN_SOLENOID, LOW); //Safety - Always deactivate the solenoid - Can't do this gotta be asyncronous to play NINE! NINE! NINE!

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
    } 
    else {
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
    } 
    else {
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

  } 
  else { // Not turning a new LED on, see if we need to cross-fade
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
  if(TDEBUG == 2){
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


unsigned char readyToPlayNextMP3(unsigned int timeInDeciseconds = 0){
  unsigned char result;
  if((timeLastFileCompleted != 0) &&
    ((ctr_time - timeLastFileCompleted) > (timeInDeciseconds*100)) &&
    !soundFileActive){
    return TRUE;
  }
  return FALSE;
} 

//Return true if file plays, else false
unsigned char playFile(unsigned char fileNumber, unsigned char delaySinceLastPlayed){
  unsigned char error = FALSE;

  //Don't send non-file commands or random file commands
  if((fileNumber > 199) || (fileNumber == 0)){
    return FALSE;
  } 

  //Play file if ready
  if(readyToPlayNextMP3(delaySinceLastPlayed)){  
    Serial.write(fileNumber);
    timeLastFileCompleted = 0; //update timer
    return TRUE;
  }
  return FALSE;
}

void whackSolenoid(){

  if((ctr_time -  solenoidTimer) > 200 && (ctr_time -  solenoidTimer) < 400){
    digitalWrite(PIN_SOLENOID, HIGH);
  }else if((ctr_time -solenoidTimer) > 400){
    solenoidTimer = 0;
    digitalWrite(PIN_SOLENOID, LOW);
    machineState = ALIGN_PLAYER_TO_MIRROR; //
  }else{
    digitalWrite(PIN_SOLENOID, LOW);
  }
}

void flashMirrorLight(){
  if((ctr_time -  mirrorLightTimer) < 200){
    digitalWrite(PIN_MIRROR_LIGHTS, HIGH);
  }
  else if((ctr_time -  mirrorLightTimer) < 400){
    digitalWrite(PIN_MIRROR_LIGHTS, LOW);
  }
  else if((ctr_time -  mirrorLightTimer) < 800){
    digitalWrite(PIN_MIRROR_LIGHTS, HIGH);
  }
  else if((ctr_time -  mirrorLightTimer) < 1200){
    digitalWrite(PIN_MIRROR_LIGHTS, LOW);
  }else{
    mirrorLightTimer = 0;
    machineState = NO_PLAYER_DETECTED;
    digitalWrite(PIN_MIRROR_LIGHTS, LOW);
  }
}
