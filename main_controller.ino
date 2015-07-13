/*
 BM 2015 Chuckie Pinball by TonyOxide
 This code is in the public domain.
 Version 0.02

 Photoresistors being used are about 0 - 800 on the analog input
 A normal room is about 100, dark room about 30 and it will to to 0
 Use a 10k resistor to ground with them to have them work properly

 */

// Variables you might want to change
unsigned long delay_red_green_min = 4000; // minimum time red/green is on
unsigned long delay_red_green_max = 10000; // maximum time red/green is on
unsigned long delay_fade = 1000; // Fade between red/green/uv
unsigned long delay_led_cross = 100; // Minimum amount of time to increment 1 crossfade value in us
unsigned char random_uv_led_pool = 5; // 1 out of every xx cycles, light the UV LED instead
unsigned int sen_light_min = 100; // Minimum value for light sensor

// Variables you probably don't want to change
unsigned long ctr_time; // Time since boot in ms
unsigned long ctr_time_prv; // Time of the start of the previous loop

unsigned char loop_delay = 50; // Default loop delay in ms

unsigned char pin_led_red = 9;     // Red LED pin
unsigned char pin_led_green = 10;  // Green LED pin
unsigned char pin_led_uv = 11;     // UV LED pin
unsigned char pin_led_cross_on = pin_led_red;  // Crossfade on/off value
unsigned char pin_led_cross_off = pin_led_green;     // Crossfade on/off value
unsigned long led_level_change = 0; // Value to change for LED
unsigned long led_level_change_prv = 0; // Last time the LED level changed

unsigned char pin_sen_light = A5;  // Light sensor pin
int led_cross_on = 255; // Crossfade level for LED turning on
int led_cross_off = 255; // Crossfade level for LED turning off

unsigned int sen_light = 0; // Light sensor absolute value
unsigned char sen_light_norm = 0; // Normalized value of light sensor output
unsigned char led_bright_red = 0; // Normalized value of LED brightness
unsigned char led_bright_green = 0; // Normalized value of LED brightness
unsigned char led_bright_uv = 0; // Normalized value of LED brightness

unsigned long delay_red_green_random = 0;  // Init random delay interval

unsigned long led_on_time = 0; // Time to turn the LED on
unsigned char pin_led_on = 0; // LED to turn on
unsigned char led_bright = 0; // Brightness of the LED
unsigned char pin_led_off1 = 0; // LED to turn off
unsigned char pin_led_off2 = 0; // LED to turn off
unsigned char pin_green = 0; // Green LED pin

// Counter init
unsigned int ctr_red = 1;   // Number of red activations (
unsigned int ctr_green = 0; // Number of green activations
unsigned int ctr_uv = 0;    // Number of UV activations
unsigned int ctr_loop = 0;  // Number of loops 

unsigned long time_led_start = 0;  // Time LED went on

void setup() {
  randomSeed(analogRead(0));
  // Setup serial for music player
  Serial.begin(9600); 
}

void loop() {
  ctr_time_prv = ctr_time; // Set the previous value of the timer
  ctr_time = millis();
  ctr_loop++;
  panel_checkFade();
  
  delay(loop_delay);
}

// Check analog light sensor to set LED brightness levels
void checkLightLevels(int sen_pin = 0) {
  sen_light = analogRead(sen_pin);
  Serial.println(sen_light);
  delay(200);
  // Map light sensor to from 0 to 900 to 0 to 255 q
  if (sen_light == 0) { // Set minimum sensor value
    sen_light = sen_light_min; 
  }
  sen_light_norm = map(sen_light, 0, 900, 0, 255);  
  // Set LED value multipliers
  led_bright_red = sen_light_norm * 2.0;
  led_bright_green = sen_light_norm * 1.0;
  led_bright_uv = sen_light_norm * 1.0;
}

void panel_checkFade() { // See if we should fade to another LED for the panel
  led_on_time = ctr_time - time_led_start;
  //Serial.println("Starting check");
  delay_red_green_random = random(delay_red_green_min, delay_red_green_max);
  if ( led_on_time > delay_red_green_random) {
    time_led_start = millis();
    // See which LED to turn on
    
    if ( ctr_red == 0 && ctr_green > 1000) { // Counter rolled over, reset counters
      ctr_green = 0;
    }
    
    pin_led_cross_off = pin_led_on; // Prevoius pin that was on

    checkLightLevels(pin_sen_light); // See what ambient light we have
    if (random(random_uv_led_pool) == 1) { // Randomly display UV LED
      pin_led_cross_on = pin_led_uv;
      pin_led_on = pin_led_uv;
      led_cross_off = led_bright; // Set crossfade max value for LED turning off
      led_bright = led_bright_uv;
      ctr_uv++;
    } else {  // Not lighting UV LED, light another one...
      if ( ctr_red > ctr_green) {
        pin_led_on = pin_led_green;
        pin_led_cross_on = pin_led_green;
        led_cross_off = led_bright; // Set crossfade max value for LED turning off
        led_bright = led_bright_green;
        ctr_green++;
      } else {
        pin_led_cross_off = pin_led_red;
        pin_led_on = pin_led_red;
        led_cross_off = led_bright; // Set crossfade max value for LED turning off
        led_bright = led_bright_red;
        ctr_red++;
      }
    }
    led_cross_on = 0;  // Set crossfade value to 0 for the LED turning on
    Serial.print("led_bright= ");
    Serial.println(led_bright);
    Serial.println(led_bright_red);
    Serial.println(led_bright_green);
  
  } else { // Not turning a new LED on, see if we need to crossfade
    if (led_bright > led_cross_on) { // LED to turn on is not yet at full brightness, increase by 1 level
      // See how many levels to change the LED
      led_level_change = (millis() - led_level_change_prv) / delay_led_cross;
      led_level_change_prv = millis();
      led_cross_on += led_level_change;
      led_cross_off -= led_level_change;
      
      if (led_cross_on > led_bright) {
        led_cross_on = led_bright;
      }
      
      if (led_cross_off < 0) {
        led_cross_off = 0;
      }
      
      analogWrite(pin_led_cross_on, led_cross_on);
      analogWrite(pin_led_cross_off, led_cross_off);
      // print the results to the serial monitor:
      Serial.print("ledon=");
      Serial.print(led_cross_on);
      Serial.print("\tledoff=");
      Serial.print(led_cross_off);
      Serial.print("\tledlevel=");
      Serial.print(led_level_change);
      Serial.print("\t pin=");
      Serial.println(pin_led_cross_on);
    }
    
  }
  
}
