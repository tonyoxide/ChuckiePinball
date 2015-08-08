/*
 BM 2015 Chuckie Pinball by TonyOxide
 This code is in the public domain.
 Version 0.02
 */

// Turn debugging on or off...
// Don't even try to use #if syntax, the IDE breaks if you do
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
unsigned long delay_led_cross = 10; // Minimum amount of time to increment 1 crossfade value in ms
unsigned long random_uv_led_pool = 2000; // 1 out of every xx seconds, light the UV LED instead
unsigned char uv_led_on_time = 150; // Time to flash UV LED
unsigned char uv_led_on_buffer = 0; // Time to turn off all other LEDS before/after UV LED
unsigned int sen_light_min = 900;   // Minimum value for light sensor
unsigned char loop_delay = 2;      // Default loop delay in ms

// Variables you probably don't want to change
unsigned long ctr_time; // Time since boot in ms
unsigned long ctr_time_prv; // Time of the start of the previous loop

unsigned char pin_led_cross_on = pin_led_red;  // Crossfade on/off value
unsigned char pin_led_cross_off = pin_led_green; // Crossfade on/off value
unsigned long led_level_change = 0; // Value to change for LED
unsigned long led_level_change_prv = 0; // Last time the LED level changed

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

// This function looks for someone standing near the exhibit
int pir_check(int pin = 0) {
  int val = 0;
  val = digitalRead(pin);
  if (val == 0) {
    digitalWrite(pin_led_test,LOW);
  } else {
    digitalWrite(pin_led_test,HIGH);
  }
  if (TDEBUG >= 2) {
      Serial.print("pir=");
      Serial.println(val);
      delay(100);
  }
}


void setup() {
  randomSeed(analogRead(0));
  // Turn on Green LED
  delay_red_green_random = random(delay_red_green_min, delay_red_green_max);
  pin_led_cross_off = pin_led_red;
  ctr_red = 1;
  time_led_start = millis() - delay_red_green_random - 1; // Turn an LED on right away

if (TDEBUG >= 1) {
  Serial.begin(9600);
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
	
	// Delay at the end of the loop (optional)
  delay(loop_delay);
}

void panel_checkFade() { // See if we should fade to another LED for the panel
	led_on_time = ctr_time - time_led_start;
	// See if we need to flash the UV LED
	if (random(random_uv_led_pool) == 1) { // Randomly display UV LED
		// Turn off the other LEDs
//		 analogWrite(pin_led_cross_on, 0);
//		 analogWrite(pin_led_cross_off, 0);
		 delay(uv_led_on_buffer);
		 // Turn on UV LED
		 analogWrite(pin_led_uv, led_bright_uv);
		 delay(uv_led_on_time);
		 // Turn off UV LED
		 analogWrite(pin_led_uv, 0);
		 delay(uv_led_on_buffer);
//		 analogWrite(pin_led_cross_on, led_cross_on);
//		 analogWrite(pin_led_cross_off, led_cross_off);
		 ctr_uv++;
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
    led_level_change_prv = millis();
		if (TDEBUG >= 1) {
			Serial.print("led_bright= ");
			Serial.println(led_bright);
			Serial.println(led_bright_red);
			Serial.println(led_bright_green);
		}

  } else { // Not turning a new LED on, see if we need to cross-fade
    if (led_cross_on < led_bright || led_cross_off != 0) { // LED to turn on is not yet at full brightness, increase by 1 level
      // See how many levels to change the LED
			
      led_level_change = (millis() - led_level_change_prv) / delay_led_cross;
      if (led_level_change == 0) { // Haven't waited long enough... return
        return;
      }
      led_level_change_prv = millis();
      led_cross_on += led_level_change;
      led_cross_off -= led_level_change;

      if (led_cross_on > led_bright || led_cross_off < 0) {  // Values too big or too small, just punt
        led_cross_on = led_bright;
        led_cross_off = 0;
      }

      analogWrite(pin_led_cross_on, led_cross_on);
      analogWrite(pin_led_cross_off, led_cross_off);
      // print the results to the serial monitor:
			if (TDEBUG >= 1) {
				Serial.print("ledon=");
				Serial.print(led_cross_on);
				Serial.print("\tledoff=");
				Serial.print(led_cross_off);
				Serial.print("\tledlevel=");
				Serial.print(led_level_change);
				Serial.print("\t pin=");
				Serial.println(pin_led_cross_on);
				delay(200);
			}
    }

  }

}




