/*
 BM 2015 Chuckie Pinball by RCI and TonyOxide
 This code is in the public domain.
 Version 0.03
 
 Changelog
 v 0.03 Adding CD4067 Driver - main program flow
 */


// Turn debugging on or off...
// Don't even try to use #if syntax, the IDE breaks if you do
// Bitmask 
#define DEBUG_LEDS 1
#define DEBUG_PIRS 2
#define DEBUG_FINGERS 4
#define DEBUG_MACHINE_STATE 8
#define DEBUG_MP3 16
#define DEBUG_SERIAL 32
#define DEBUG_INNER_LED 64
//unsigned char TDEBUG = DEBUG_SERIAL | DEBUG_MACHINE_STATE | DEBUG_MP3;
unsigned char TDEBUG = 40;

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
  12,13};
unsigned char asnd_fingers[] = {
  27, 28}; 
unsigned char asnd_attract[] = {
  15,16,17,18,19.20,21,22,23,24};
unsigned char asnd_leave[] = {
  38,39};

#define TRUE  1
#define FALSE 0

//State Machine
#define NO_PLAYER_DETECTED 0
#define PLAYER_DETECTED 1
#define COUNTING_PLAYER_FINGERS 2
#define SOLENOID_ACTIVE 3
#define LAUGH_AT_PLAYER 4
#define GOTCHA 5
#define ALIGN_PLAYER_TO_MIRROR 6
#define PLAYER_ALIGNED_TO_MIRROR 7
#define LAUGH_AT_PLAYER_AGAIN 8
#define WAIT_FOR_PLAYER_TO_LEAVE 9

//Pinout
#define PIN_MIRROR_LIGHTS 6
#define PIN_SOLENOID 4

//Distance
#define DISTANCE_SENSOR_TOO_FAR 125
#define DISTANCE_SENSOR_TOO_CLOSE 160

#define FINGER_DEBOUCE_LENGTH 250 //counting in loop cycles right now

#define PLAYER_DISTANCE_TIMEOUT_LENGTH 20000 //milliseconds

// Variables you might want to change
#define pin_led_red 9      // Red LED pin
#define pin_led_green 10   // Green LED pin
#define pin_led_uv 11      // UV LED pin
#define pin_led_finger_white_red 13 // White = high, Red = low
#define pin_led_finger_blue 8 // Blue LED strip to right
#define pin_led_finger_green 12 // Green LED strip to left
#define pin_face_sen 14    // Face sensor pin
#define delay_red_green_min 5000 // minimum time red/green is on
#define delay_red_green_max 10000 // maximum time red/green is on
#define delay_fade 2000    // Fade between red/green/uv
float delay_led_cross = 600; // Minimum amount of time to increment 1 crossfade value in ms
#define random_uv_led_pool 9000 // 1 out of every xx seconds, light the UV LED instead
#define uv_led_on 850 // Time to flash UV LED
#define uv_led_on_buffer 0 // Time to turn off all other LEDS before/after UV LED
#define loop_delay 2      // Default loop delay in ms

// Variables you probably don't want to change
unsigned long uv_led_on_time = 0; // Time that the UV LED was turned on
unsigned long ctr_time = 10; // Time since boot in ms, must be 10 to play sound in the beginning
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

unsigned long fingerCountTimeout = 0;
unsigned char fingerCountTimeoutStarted = false; //Set on timeout begin

unsigned char countSpoken = FALSE;
#define FINGER_COUNT_TIMEOUT_LENGTH 15000
#define MIRROR_TIMEOUT_LENGTH 15000
#define WAIT_TO_LEAVE_TIMEOUT_LENGTH 15000
unsigned long solenoidTimer = 0;
unsigned char solenoidComplete = false;

//TCB380
unsigned char soundFileActive = false;
#define pin_tcb380Active 3    //sound file output
unsigned long  timeLastFileCompleted = 1; //Initilize so we can play on bootup

//Mirror light variables
unsigned long mirrorLightTimer = 0;
unsigned char mirrorTimerStarted = false;

unsigned long mirrorAlignmentTimeoutTimer = 0;
unsigned char mirrorAlignmentTimeoutTimerStarted = false;


//User State Variables
unsigned char machineState = NO_PLAYER_DETECTED; //Check define tablefor states
unsigned char userInProximity = 0;
unsigned int  userDistance = 0;
unsigned int fingerDebounceCount = 0;
unsigned long playerDetectedTime = 0;
unsigned char playerDetectedTimeoutStarted = false;

unsigned char waitForPlayerToLeaveTimeoutStarted = false;
unsigned long waitForPlayerToLeaveTimeout = 0;

//Sound file flags
unsigned char timeToLaugh = false;
unsigned char laughPlayed = false;
unsigned char distanceAchievedPlayed = false;
unsigned char nowLookIntoMyMirrorPlayed = false;
unsigned char selectRandomTaunt = 0;
unsigned char selectRandomLeavePhrase = 0;
unsigned char selectRandomLaugh = 0;
unsigned char leaveFilePlayed = false;
unsigned char selectRandomFingersPhrase = 0;
unsigned char lastFilePlaySuccess = false;
// Finger light flags
unsigned long flStartTime = 0;
#define flFlashOnTime 500
#define flFlashOffTime 250
#define flFlashCount 5
// Case light flags
unsigned long caseFlashTime = 0;
#define caseFlashRate 7 // This is a binary shift so 8 = 256ms, 7 = 128ms, 6 = 64ms;

void setup() {
  randomSeed(analogRead(0));
  // Turn on Green LED
  delay_red_green_random = random(delay_red_green_min, delay_red_green_max);
  pin_led_cross_off = pin_led_red;
  ctr_red = 1;
	ctr_time = 10; // Need this for playing debug before main loop
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
  delay(100);
  pinMode(pin_tcb380Active, INPUT);
  pinMode(pin_led_finger_white_red, OUTPUT);
  pinMode(pin_led_finger_green, OUTPUT);
  pinMode(pin_led_finger_blue, OUTPUT);

  //POST Routine
  //led_test();
  //lightBoxTest();
  //solenoidTest();
  //playFile(SND_DONECOUNT, 0);
  //debugSay(123);
  playFile(SND_TWOFACE, 0); //Play something at start up since the bark takes a while
}

void loop() {
  // Housekeeping
  ctr_time_prv = ctr_time; // Set the previous value of the timer
  ctr_time = millis();
  time_loop_delay = ctr_time - ctr_time_prv; // Delay since the last loop
  ctr_loop++;

  // See what we should do with the illumination LEDs
  panel_checkFade();

	flashRedFingerLight(FALSE);
	
// Main internal LED states
// Attract - both on
// Count - green
// Mirror - blue
// Wait - wait for player to leave - All off
	
	switch (machineState) {
		case PLAYER_DETECTED:
  		led_internal_green_on(true);
  		led_internal_blue_on(true);
			break;
			
		case NO_PLAYER_DETECTED:
  		led_internal_green_on(true);
  		led_internal_blue_on(true);
			break;
			
		case COUNTING_PLAYER_FINGERS:
  		led_internal_green_on(true);
  		led_internal_blue_on(false);
			break;
			
		case ALIGN_PLAYER_TO_MIRROR:
  		led_internal_green_on(false);
  		led_internal_blue_on(true);
			break;
		
		case LAUGH_AT_PLAYER_AGAIN:
			led_internal_green_on(false);
  		led_internal_blue_on(true);
			break;
			
		case WAIT_FOR_PLAYER_TO_LEAVE:
  		led_internal_green_on(false);
  		led_internal_blue_on(false);
			break;
			
		default:
  		led_internal_green_on(false);
  		led_internal_blue_on(false);
			break;
	}
	
  //If we're in serial debug we need to skip the input read routines
  if(!(TDEBUG & 32)){
    //Scan the mux to count fingers
    lastFingerCount = totalFingersCounted;
  
    totalFingersCounted = scanFingerSensors();
    if(lastFingerCount != totalFingersCounted){
      fingerDebounceCounter = millis();
    }
		
    userInProximity = readFrontPIRSensor(); //userInProximity holds the PIR Sensor
    userDistance = readDistanceSensor();  //userDistance is the distance!
  }
	
  //Update time when the busy line transitions to file inactive
  if(soundFileActive == TRUE && !is_sound_playing()){
    if(!(TDEBUG & 32)){
      timeLastFileCompleted = millis(); // use the millis function, main loop distance might impair ctr_time
    }
  }
	
  //update the file active states
  soundFileActive = is_sound_playing();
	
  //No user detected - Bark for attention  
  selectRandomTaunt = random(sizeof(asnd_attract));
  if(machineState == NO_PLAYER_DETECTED){
    /*if(TDEBUG & DEBUG_SERIAL){
      Serial.println("Taunt players!");
    }*/
    playFile(asnd_attract[selectRandomTaunt], 15000);
  }
	
  //Finger count routine begins - overrides PIR   
  if((totalFingersCounted > 0) &&
    ((machineState == NO_PLAYER_DETECTED) ||
    (machineState == PLAYER_DETECTED)))
  {
    machineState = COUNTING_PLAYER_FINGERS;
    
    if(fingerCountTimeoutStarted == false){
      playerDetectedTimeoutStarted = false; //Gotta reset the flag if we jump into the counter
      fingerCountTimeout = millis(); //Initiate timeout counter
      fingerCountTimeoutStarted = true; //Set timeout flag
          
      if(TDEBUG & DEBUG_MACHINE_STATE){
        Serial.print("state: ");
        Serial.println(machineState);
      }      
    }
    
        
    if(TDEBUG & DEBUG_SERIAL){
      Serial.println("Counting Fingers");
      Serial.print("Counting Timeout: ");
      Serial.println(fingerCountTimeout);
    }
  }

  //Debounce finger input sensors
  //until the number of fingers is stable
  if((machineState == COUNTING_PLAYER_FINGERS) && (totalFingersCounted < 10))
  {    
    //If the timeout counter has been initiated, check against timeout length
    if(fingerCountTimeoutStarted == true){
      if(checkForTimeout(fingerCountTimeout, FINGER_COUNT_TIMEOUT_LENGTH)){ //the timeout has been reached        
        lastFilePlaySuccess = playFile(SND_DONECOUNT, 100); //Tell the player the count is done
        if(lastFilePlaySuccess){
          fingerCountTimeoutStarted = false; //reset flag
          machineState = ALIGN_PLAYER_TO_MIRROR; //go on to mirror alignment
          lastFilePlaySuccess = false;
          
          if(TDEBUG & DEBUG_MACHINE_STATE){
            Serial.print("state: ");
            Serial.println(machineState);
          }          
        }        
      } else {
				playFile(totalFingersCounted, 100); //For zero, play file will return false
      }
    }

    /*if(TDEBUG & 16){
     Serial.println((totalFingersCounted-1) + 48);
     }*/
  }
	
  if((machineState == COUNTING_PLAYER_FINGERS) && (totalFingersCounted == 10) && !timeToLaugh){
    lastFilePlaySuccess = playFile(totalFingersCounted, 100);  //Max fingers achieved
    if(lastFilePlaySuccess){
      lastFilePlaySuccess = false;
      solenoidTimer = millis();
      machineState = SOLENOID_ACTIVE;
      fingerCountTimeoutStarted = false; //reset flag
      if(TDEBUG & DEBUG_SERIAL){
        Serial.println("Whack!");
      }
      
      if(TDEBUG & DEBUG_MACHINE_STATE){
        Serial.print("state: ");
        Serial.println(machineState);
      }      
    }
  }    
  
  //Play Nine, Nine, Nine! while solenoid is running
  if(machineState == SOLENOID_ACTIVE){
    
    if(!lastFilePlaySuccess){
      lastFilePlaySuccess = playFile(SND_NINE, 300); //Minus one HAHAHAHA
    }
    
    if(whackSolenoid() && lastFilePlaySuccess){
      machineState = LAUGH_AT_PLAYER;
      lastFilePlaySuccess = false; //reset flag
      fingerCountTimeoutStarted = false; //reset flag
      if(TDEBUG & DEBUG_SERIAL){
        Serial.println("Nine Nine Nine!");
      }
      if(TDEBUG & DEBUG_MACHINE_STATE){
        Serial.print("state: ");
        Serial.println(machineState);
      }      
    }
  }
  
  //Have a laugh at the player
  if(machineState == LAUGH_AT_PLAYER){
		flashRedFingerLight(TRUE);
    selectRandomLaugh = random(sizeof(asnd_laugh));
    lastFilePlaySuccess = playFile(asnd_laugh[selectRandomLaugh], 500);

    //Laugh complete - clear flag
    if(lastFilePlaySuccess){
      lastFilePlaySuccess = false; //reset flag
      machineState = GOTCHA;
      if(TDEBUG & DEBUG_SERIAL){
        Serial.println("Ha Ha Ha Ho Ho HO!");
      }
    }
  }

  //Got you good, player
  if(machineState == GOTCHA){
    
    lastFilePlaySuccess = playFile(SND_GOTCHA, 300);

    //Laugh complete - clear flag
    if(lastFilePlaySuccess){
      lastFilePlaySuccess = false; //reset flag
      machineState = ALIGN_PLAYER_TO_MIRROR;
      if(TDEBUG & DEBUG_SERIAL){
        Serial.println("Gotcha!");
      }
      
      if(TDEBUG & DEBUG_MACHINE_STATE){
        Serial.print("state: ");
        Serial.println(machineState);
      }      
    }
  }

  //If the front PIR detects movement //Not implemented yet -- OR the distance sensor is below threshold
  //if((machineState == NO_PLAYER_DETECTED) && userInProximity && !playerDetectedTimeoutStarted){    
    if((machineState == NO_PLAYER_DETECTED) && userInProximity){
    
    lastFilePlaySuccess = playFile(SND_COUNT, 0); //Let me count!
    
    if(lastFilePlaySuccess){
       lastFilePlaySuccess = false;
    machineState = PLAYER_DETECTED;
       if(TDEBUG & DEBUG_MACHINE_STATE){
          Serial.print("state: ");
          Serial.println(machineState);
        }
    }
    
    if(playerDetectedTimeoutStarted == false){ //if the timeout hasn't been initiated
      playerDetectedTime = millis();  //Start timeout counter
      playerDetectedTimeoutStarted = true; //Set flag
    }
  }

  //Player in the area - Bark at them to come over
  if(machineState == PLAYER_DETECTED){
     if(checkForTimeout(playerDetectedTime, PLAYER_DISTANCE_TIMEOUT_LENGTH)){ //if the timeout hits, go back to no player detected
       lastFilePlaySuccess = playFile(SND_SCARED,1000);
       
       if(lastFilePlaySuccess){
         machineState = NO_PLAYER_DETECTED;
         playerDetectedTimeoutStarted = false;
         lastFilePlaySuccess = false;
          if(TDEBUG & DEBUG_MACHINE_STATE){
            Serial.print("state: ");
            Serial.println(machineState);
          }         
        }        

       if(TDEBUG & DEBUG_SERIAL){
         Serial.println("PIR Detect Timeout!");         
       }
     }else{ //Timeout hasn't been reached, instruct player what to do
       selectRandomFingersPhrase = random(sizeof(asnd_fingers));
       playFile(asnd_fingers[selectRandomFingersPhrase], 4000);
     }
  }
  
  //Add timer to call the user a spoilsport if they won't come

  //Begin Magic Mirror routine
  if(machineState == ALIGN_PLAYER_TO_MIRROR && !nowLookIntoMyMirrorPlayed){
    nowLookIntoMyMirrorPlayed = playFile(31, 1000);
    
    if(mirrorAlignmentTimeoutTimerStarted == false){
      mirrorAlignmentTimeoutTimer = millis();
      mirrorAlignmentTimeoutTimerStarted = true;
    }
  }
  else if((machineState == ALIGN_PLAYER_TO_MIRROR) && nowLookIntoMyMirrorPlayed){
  //Guide user to optimal distance  
             
    //If above threshold - speak "closer"
    if(userDistance < DISTANCE_SENSOR_TOO_FAR){
      playFile(0x20, 1500);
    }
    else if(userDistance > DISTANCE_SENSOR_TOO_CLOSE){
      playFile(0x21, 1500);
    }
    else{ //User is the right distance
      distanceAchievedPlayed = playFile(0x22, 1000);
      if(distanceAchievedPlayed){
        distanceAchievedPlayed = false; //reset flag
        nowLookIntoMyMirrorPlayed = false; //reset flag
        mirrorAlignmentTimeoutTimerStarted = false; //reset flag        
        machineState = PLAYER_ALIGNED_TO_MIRROR;
        //mirrorLightTimer = millis();
      }
    }   
    
    //If the timeout counter has been initiated, check against timeout length
    if(mirrorAlignmentTimeoutTimerStarted == true){
      if(checkForTimeout(mirrorAlignmentTimeoutTimer, MIRROR_TIMEOUT_LENGTH)){ //the timeout has been reached        
        lastFilePlaySuccess = playFile(SND_DONEMIRROR, 100); //Tell the player the count is done
        if(lastFilePlaySuccess){
          distanceAchievedPlayed = false; //reset flag
          nowLookIntoMyMirrorPlayed = false; //reset flag
          mirrorAlignmentTimeoutTimerStarted = false; //reset flag
          machineState = WAIT_FOR_PLAYER_TO_LEAVE; //go on to mirror alignment
          lastFilePlaySuccess = false;
          
          if(TDEBUG & DEBUG_MACHINE_STATE){
            Serial.print("state: ");
            Serial.println(machineState);
          }
        }
				  if(TDEBUG & DEBUG_SERIAL){
            Serial.print("Done with Mirror: ");
            Serial.println(lastFilePlaySuccess);
          }
      }    
    }
  }
    
  //Instruct user to find their true selves
  if(machineState == PLAYER_ALIGNED_TO_MIRROR){
    if(mirrorTimerStarted == FALSE){
      mirrorLightTimer = millis();
      mirrorTimerStarted = TRUE;
    }
    if(flashMirrorLight()){ //flashMirrorLight will return true once it's complete
      machineState = LAUGH_AT_PLAYER_AGAIN;
      if(TDEBUG & DEBUG_MACHINE_STATE){
        Serial.print("state: ");
        Serial.println(machineState);
      }
    }
  }
  
  //Have another laugh at the player
  if(machineState == LAUGH_AT_PLAYER_AGAIN){
    selectRandomLaugh = random(sizeof(asnd_laugh));
    lastFilePlaySuccess = playFile(asnd_laugh[selectRandomLaugh], 1000);

    //Laugh complete - clear flag
    if(lastFilePlaySuccess){
      lastFilePlaySuccess = false; //reset flag
      machineState = WAIT_FOR_PLAYER_TO_LEAVE;
      if(TDEBUG & DEBUG_SERIAL){
        Serial.println("Ha Ha Ha Ho Ho HO!");
      }
      if(TDEBUG & DEBUG_MACHINE_STATE){
        Serial.print("state: ");
        Serial.println(machineState);
      }
    }
  }  

  if(machineState == WAIT_FOR_PLAYER_TO_LEAVE){
    selectRandomLeavePhrase = random(sizeof(asnd_leave));
    //leaveFilePlayed = 
    
    if(waitForPlayerToLeaveTimeoutStarted == false){
      waitForPlayerToLeaveTimeout = millis();
      waitForPlayerToLeaveTimeoutStarted = true;
      //playFile(asnd_leave[selectRandomLeavePhrase], 2000);
    }
    
    /*if(lastFilePlaySuccess == false){
      lastFilePlaySuccess = playFile(asnd_leave[selectRandomLeavePhrase], 2000);      
      if(TDEBUG & DEBUG_MACHINE_STATE){
        Serial.print("state: ");
        Serial.println(machineState);
      }
    }*/
    
    playFile(asnd_leave[selectRandomLeavePhrase], 5000);
    
    //If the timeout counter has been initiated, check against timeout length
    if(waitForPlayerToLeaveTimeoutStarted == true){
      if(checkForTimeout(waitForPlayerToLeaveTimeout, WAIT_TO_LEAVE_TIMEOUT_LENGTH)){ //the timeout has been reached        
          playerDetectedTimeoutStarted = false; //Couldn't find a better place for this flag change. Allow player detection again.
          machineState = NO_PLAYER_DETECTED; //Back to start of the state machine
          lastFilePlaySuccess = false;
          waitForPlayerToLeaveTimeoutStarted = false; //reset flag
          
          resetAllStateFlags(); //All the flags should already be cleared right?!?! Well in case one got missed...
          if(TDEBUG & DEBUG_MACHINE_STATE){
            Serial.print("state: ");
            Serial.println(machineState);
          }          
      }                         
    }
  }

  //Timer for user to leave
  //Tell them to scram on timeout
  //Back to  outer loop

  //Serial.println(userDistance);
  /*if(TDEBUG & DEBUG_MACHINE_STATE){
    Serial.print("state: ");
    Serial.println(machineState);
  }*/
  //Check Rear PIR - yell at hooligans behind the machine

  //digitalWrite(PIN_SOLENOID, LOW); //Safety - Always deactivate the solenoid - Can't do this gotta be asyncronous to play NINE! NINE! NINE!
  
  //If in serial debug mode
  if(TDEBUG & DEBUG_SERIAL){
    serialDebugRead(); //Check for new serial command
  }  

  // Delay at the end of the loop (optional)
  delay(loop_delay);
}

void flashRedFingerLight(unsigned char beginFlash) {  // If this is the beginning of a flash, set this flag
	unsigned long onTime,onTimeEnd,offTimeEnd;
	
	if (beginFlash && flStartTime == 0) { // Reset timer and turn the light on
		flStartTime = ctr_time;
		digitalWrite(pin_led_finger_white_red, LOW);
		return;
	} else if (flStartTime > 0) { // Currently flashing
		unsigned long flashTime = ctr_time - flStartTime; // Time we have flashed so far
		//debugSay(flashTime);
		//delay(5000);
		// See if we should transition
		unsigned long flCtr;
		for (flCtr=0; flCtr<flFlashCount; flCtr++) {
			onTime = flCtr * (flFlashOnTime + flFlashOffTime);
			onTimeEnd = onTime + flFlashOnTime;
			offTimeEnd = onTimeEnd + flFlashOffTime;
			
			if (flashTime >= onTime && flashTime <= onTimeEnd) { // Turn it on
				digitalWrite(pin_led_finger_white_red, LOW);
				return;
			} else if (flashTime > onTimeEnd && flashTime < offTimeEnd) { // Turn it off
				digitalWrite(pin_led_finger_white_red, HIGH);
				return;
			}
		}
		// Made it to the end of the loop....
		//debugSay(onTime);
		//delay(3000);
		// See if we should end the loop
		unsigned long flashTotalTime = flFlashCount * (flFlashOffTime + flFlashOnTime); 	// Total time we will flash
		if (flashTime > flashTotalTime) { // Reset the LED to white and the reset counter
			digitalWrite(pin_led_finger_white_red, HIGH);
			flStartTime = 0;
			return;
		}
	} else { // Just turn on the white LED
		digitalWrite(pin_led_finger_white_red, HIGH);
	}
	// Do nothing because beginFlash is false and we aren't flashing...
}

unsigned char is_sound_playing() {  // Returns true if there is a sound playing
	 if (digitalRead(pin_tcb380Active) == 0) { //Active low from mp3 module while sound is playing
		return TRUE;
	 } else {
		 return FALSE;
	 }
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
  if(TDEBUG & 2){
    Serial.println("");
  }
  return tempFingerCount;
}

//Check the Front PIR Sensor
unsigned char readFrontPIRSensor(){
  unsigned char result = 0;

  result = readMuxPin(10);
  if(TDEBUG & 4){
    Serial.print("PIR = ");
    Serial.println(result);
  }
  return result;

}

//Check the distance sensor
unsigned char readDistanceSensor(){
  unsigned char result = 0;

  result = readMuxPin(12);

  if(TDEBUG & 4){
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
    delay(20);
    result = analogRead(muxInputPin);
    break;      

  default:
    break;
  }

  return result;
}

// Test functions
void led_test() { // Startup test for LEDs
	for(char x=0;x<10;x++){
		analogWrite(pin_led_red, 255);
		analogWrite(pin_led_green, 255);
		analogWrite(pin_led_uv, 255);
		digitalWrite(pin_led_finger_white_red, LOW);
		digitalWrite(pin_led_finger_blue, HIGH);
		digitalWrite(pin_led_finger_green, HIGH);
		delay(350);
		analogWrite(pin_led_red, 0);
		analogWrite(pin_led_green, 0);
	  analogWrite(pin_led_uv, 0);
		digitalWrite(pin_led_finger_white_red, HIGH);
		digitalWrite(pin_led_finger_blue, LOW);
		digitalWrite(pin_led_finger_green, LOW);
		delay(350);
	}
}

void lightBoxTest(){
	for(char x=0;x<10;x++){
	  digitalWrite(PIN_MIRROR_LIGHTS, HIGH);
		delay(250);
		digitalWrite(PIN_MIRROR_LIGHTS, LOW);
		delay(250);
	}
}

void solenoidTest(){
	for(char x=0;x<3;x++){
	 digitalWrite(PIN_SOLENOID, HIGH);
	 delay(700);
	 digitalWrite(PIN_SOLENOID, LOW);
	 delay(300);
	}
}


unsigned char readyToPlayNextMP3(unsigned long timeInMilliseconds){

  unsigned long currentTime = millis();
  
  if((timeLastFileCompleted != 0) && ((currentTime - timeLastFileCompleted) > timeInMilliseconds) && !is_sound_playing()){
    if(TDEBUG & 16){
      Serial.println('timer : %l', ctr_time);
      Serial.println('time last completed: %l', timeLastFileCompleted);
    }
    return TRUE;
  }
  return FALSE;
} 

//Return true if file plays, else false
unsigned char playFile(unsigned char fileNumber, unsigned long delaySinceLastPlayed){
 
  unsigned long  currentTime = millis();
  
  //Don't send non-file commands or random file commands
  if((fileNumber > 199) || (fileNumber == 0)){
    return FALSE;
  } 

  //Play file if ready
  if(TDEBUG & DEBUG_MP3 || TDEBUG & DEBUG_SERIAL) { 
    //Adding timer from readyToPlayNextMP3 to Serial debug code
    //Not good to nest these if's but it makes more sense to me
    if((currentTime - timeLastFileCompleted) > delaySinceLastPlayed){  
      Serial.write("playFile: ");
      Serial.println(fileNumber);
      /*Serial.print("timer: ");
      Serial.println(currentTime);
      Serial.print("time last completed: ");
      Serial.println(timeLastFileCompleted);*/
      timeLastFileCompleted = currentTime; //update timer      
      return TRUE; //Return true if the time condition has been met
    }
  }
  else if(readyToPlayNextMP3(delaySinceLastPlayed)){
    Serial.write(fileNumber);    
    timeLastFileCompleted = currentTime; //update timer      
    return TRUE;
  }
  return FALSE;
}

//timer for the solenoid
//returns true when it's complete
unsigned char whackSolenoid(){
  
 unsigned long currentTime = millis();

  if((currentTime - solenoidTimer) < 1400){  // Wait until ten sound file plays
		digitalWrite(PIN_SOLENOID, LOW);
	} else if((currentTime - solenoidTimer) < 1800) {
		digitalWrite(PIN_SOLENOID, HIGH);
  }
  else{
    if(TDEBUG & DEBUG_MACHINE_STATE){
      Serial.print("state: ");
      Serial.println(machineState);
    }      
    digitalWrite(PIN_SOLENOID, LOW);
    return true;
  }
  
  return false;
}

//timer for the mirror lights
//returns true when it's complete
unsigned char flashMirrorLight(){
  unsigned long currentTime = millis();
  
  if((currentTime -  mirrorLightTimer) < 3200){ //Wait until alignment file plays
    digitalWrite(PIN_MIRROR_LIGHTS, LOW);
  }
  else if((currentTime -  mirrorLightTimer) < 3250){
    digitalWrite(PIN_MIRROR_LIGHTS, HIGH);
  }
  else if((currentTime -  mirrorLightTimer) < 3500){
    digitalWrite(PIN_MIRROR_LIGHTS, LOW);
  }
  else if((currentTime -  mirrorLightTimer) < 3750){
    digitalWrite(PIN_MIRROR_LIGHTS, HIGH);
  }
  else if((currentTime -  mirrorLightTimer) < 4000){
    digitalWrite(PIN_MIRROR_LIGHTS, LOW);
  }
  else if((currentTime -  mirrorLightTimer) < 4250){
    digitalWrite(PIN_MIRROR_LIGHTS, HIGH);
  }else{
    
    mirrorTimerStarted = false; //Reset timer flag

    digitalWrite(PIN_MIRROR_LIGHTS, LOW);
    if(TDEBUG & 32){
      Serial.println("Mirror Flash Complete!");
    }
    return true;
  }
  
  return false;
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

		while (playFile(fileNumber,0) == FALSE) {
			delay(10);  // Make sure the file gets played
		}
		timeLastFileCompleted = 1; // Allow immediate playing of next sound...
		delay(500);
}

//Set TDEBUG to serial debug and uses commands below to
//interact with top level state machine
void serialDebugRead(){
  unsigned char incomingByte = 0;
  
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();

                if(incomingByte == 'P'){
                  userInProximity = !userInProximity;
                }
                else if(incomingByte == 'T'){
                  totalFingersCounted = 10;
                }
                else if(incomingByte == 'D'){
                  userDistance = 140;
                }
                
                // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte, DEC);
        }
}

//Generic time compare function, in milliseconds
//Returns true if timeoutDuration has elapsed since intialTime
unsigned char checkForTimeout(unsigned long initialTime, unsigned long timeoutDuration){

  unsigned long currentTime = millis();
  
   if(TDEBUG & DEBUG_SERIAL){
      Serial.println("Check For Timeout");
      Serial.print("Current time: ");
      Serial.println(currentTime);
      Serial.print("Initial Time: ");
      Serial.println(initialTime);
			Serial.print("mirrorAlignmentTimeoutTimerStarted: ");
			Serial.println(mirrorAlignmentTimeoutTimerStarted);
    }

  if((currentTime -  initialTime) >= timeoutDuration){
    return true;
  }else{
    return false;
  }
}

void resetAllStateFlags(){
  countSpoken = false;
  //Mirror light variables
  mirrorTimerStarted = false;
  mirrorAlignmentTimeoutTimerStarted = false;
  
  
  //User State Variables
  playerDetectedTimeoutStarted = false;
  
  waitForPlayerToLeaveTimeoutStarted = false;
  
  //Sound file flags
  timeToLaugh = false;
  laughPlayed = false;
  distanceAchievedPlayed = false;
  nowLookIntoMyMirrorPlayed = false;
  leaveFilePlayed = false;
  lastFilePlaySuccess = false;  
}

void led_internal_green_on(unsigned char turnOn) {
	if (turnOn) {
		// See if we should turn on or off
		if ((ctr_time >> caseFlashRate) % 2 == 1) {
			if (TDEBUG & DEBUG_INNER_LED) {
        Serial.print("on gr: ");
        Serial.println(ctr_time);
        delay(20);
      }
			digitalWrite(pin_led_finger_green, HIGH);
		}	else {
			if (TDEBUG & DEBUG_INNER_LED) {
        Serial.print("of gr: ");
        Serial.println(ctr_time);
        delay(20);
      }
			digitalWrite(pin_led_finger_green, LOW);
		} 
	} else { // Turn off
			if (TDEBUG & DEBUG_INNER_LED) {
        Serial.print("o2 gr: ");
        Serial.println(ctr_time);
        delay(20);
      }
		digitalWrite(pin_led_finger_green, LOW);
	}
}

void led_internal_blue_on(unsigned char turnOn) {
	if (turnOn) {
		// See if we should turn on or off
		if ((ctr_time >> caseFlashRate) % 2 == 0) {
			if (TDEBUG & DEBUG_INNER_LED) {
        Serial.print("on bl: ");
        Serial.println(ctr_time);
        delay(20);
      }
			digitalWrite(pin_led_finger_blue, HIGH);
		}	else {
			if (TDEBUG & DEBUG_INNER_LED) {
        Serial.print("of bl: ");
        Serial.println(ctr_time);
        delay(20);
      }
			digitalWrite(pin_led_finger_blue, LOW);
		} 
	} else { // Turn off
			if (TDEBUG & DEBUG_INNER_LED) {
        Serial.print("o2 bl: ");
        Serial.println(ctr_time);
        delay(20);
      }
		digitalWrite(pin_led_finger_blue, LOW);
	}
}
