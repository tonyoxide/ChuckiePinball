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
unsigned char asnd_laugh[] = {
  11,12,13,14};
unsigned char asnd_attract[] = {
  14,15,16,17,18,19.20,21,22,23,24};
unsigned char asnd_leave[] = {
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

unsigned char TDEBUG = 0;

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
#define pin_tcb380Active 3    //sound file output
unsigned int  timeLastFileCompleted = 1; //Initilize so we can play on bootup


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
  pinMode(pin_tcb380Active, INPUT);

  // led_test();
  lightBoxTest();
  // solenoidTest();

  pinMode(pin_led_test, OUTPUT);

	digitalWrite(pin_led_test,HIGH);
	delay(300);
	digitalWrite(pin_led_test,LOW);
	delay(300);
			
  debugSay(7451);
//	delay(1000);
  playDigit(1);
  playDigit(2);
  playDigit(3);
  playDigit(4);
  playDigit(5);
  playDigit(6);
  playDigit(7);
  playDigit(8);
  playDigit(9);
  playDigit(0);
	
	lightBoxTest();
}

void loop() {
	digitalWrite(pin_led_test,HIGH);
	delay(200);
	digitalWrite(pin_led_test,LOW);
	delay(200);
  // Housekeeping
	// Re-init RNG periodically
	if (ctr_time % 1000 == 1) {
		randomSeed(analogRead(0));
	}
  ctr_time_prv = ctr_time; // Set the previous value of the timer
  ctr_time = millis();
  time_loop_delay = ctr_time - ctr_time_prv; // Delay since the last loop
  ctr_loop++;


  // Delay at the end of the loop (optional)
  delay(loop_delay);
}

unsigned char is_sound_playing() {  // Returns true if there is a sound playing
	 if (digitalRead(pin_tcb380Active) == 0) { //Active low from mp3 module while sound is playing
		 return TRUE;
	 } else {
		 return FALSE;
	 }
}


void lightBoxTest(){
  digitalWrite(PIN_MIRROR_LIGHTS, HIGH);
  delay(1000);
  digitalWrite(PIN_MIRROR_LIGHTS, LOW);
}

unsigned char readyToPlayNextMP3(unsigned long timeInMilliseconds){

  if((timeLastFileCompleted != 0) && ((ctr_time - timeLastFileCompleted) > timeInMilliseconds) && !is_sound_playing()){
    return TRUE;
  }
  digitalWrite(pin_led_test,HIGH);
	delay(500);
	digitalWrite(pin_led_test,LOW);
	delay(500);
	digitalWrite(pin_led_test,HIGH);
	delay(500);
	digitalWrite(pin_led_test,LOW);
	delay(500);
} 

//Return true if file plays, else false
unsigned char playFile(unsigned char fileNumber, unsigned long delaySinceLastPlayed){
 
  //Don't send non-file commands or random file commands
  if((fileNumber > 199) || (fileNumber <= 0)){
    return FALSE;
  } 

  //Play file if ready
  if(readyToPlayNextMP3(delaySinceLastPlayed)){  
    Serial.write(fileNumber);
    timeLastFileCompleted = 0; //update timer
    delay(1);
    return TRUE;
  }
	digitalWrite(pin_led_test,HIGH);
	delay(500);
	digitalWrite(pin_led_test,LOW);
	delay(500);
	digitalWrite(pin_led_test,HIGH);
	delay(500);
	digitalWrite(pin_led_test,LOW);
	delay(500);
  return FALSE;
}


void debugSay(unsigned int num) { // Say each digit of a number
	// Break the number into multiple parts
	unsigned int ones,tens,hund,thou,tthou;

	tthou = num/10000;
	if (tthou > 0) {
		playDigit(tthou);
	}
	thou = (num-(tthou*10000))/1000;
	if (thou > 0 || tthou > 0) {
		playDigit(thou);
	}
	hund = (num-(tthou*10000+thou*1000))/100;
	if (hund > 0 || tthou > 0 || thou > 0) {
		playDigit(hund);
	}
	tens = (num-(tthou*10000+thou*1000+hund*100))/10;
	if (tens > 0 || hund > 0 || tthou > 0 || thou > 0) {
		playDigit(tens);
	}
	ones = num-(tthou*10000+thou*1000+hund*100+tens*10);
	playDigit(ones);
}

void playDigit(unsigned char num) { // Say a single digit
	unsigned char fileNumber;
	switch(num) {
		case 0:
			fileNumber = SND_10;
			break;
		
		case 1:
			fileNumber = SND_1;
			break;
		
		case 2:
			fileNumber = SND_2;
			break;
		
		case 3:
			fileNumber = SND_3;
			break;
		
		case 4:
			fileNumber = SND_4;
			break;
		
		case 5:
			fileNumber = SND_5;
			break;
		
		case 6:
			fileNumber = SND_6;
			break;
		
		case 7:
			fileNumber = SND_7;
			break;
		
		case 8:
			fileNumber = SND_8;
			break;
		
		case 9:
			fileNumber = SND_9;
			break;
	}
	// Wait for us to be finished playing the last sound
		while (is_sound_playing()) { // Playing a sound, wait
			digitalWrite(pin_led_test,HIGH);
			delay(10);
		}
		digitalWrite(pin_led_test,LOW);
		playFile(fileNumber,10);
}
