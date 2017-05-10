//For Usage of the Adafruit Neopixel Rings
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

//For Usage of the Haptic Controler
#include "Adafruit_DRV2605.h"

//Small Ring
#define PIN 7
#define NUM_PIXELS 12
Adafruit_NeoPixel ring = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

//Big Ring
#define PIN_BIG_RING 8
#define BIG_RING_NUM_PIXELS 24
Adafruit_NeoPixel big_ring = Adafruit_NeoPixel(60, PIN_BIG_RING, NEO_GRB + NEO_KHZ800);


/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_DRV2605 drv;

//  VARIABLES
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
//int blinkPin = 13;                // pin to blink led at each beat

// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.



const int leftButton = 2;
const int upButton = 3;
const int rightButton = 4;
const int downButton = 5;
const int ledPin =  13;
#define potPin  "A5"
int buttonState = 0;
#define CDS_INPUT 0 

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(leftButton, INPUT);
  pinMode(upButton, INPUT);
  pinMode(rightButton, INPUT);
  pinMode(downButton, INPUT);
  Serial.begin(9600);
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  ring.begin();
  ring.show(); // Initialize all pixels to 'off'

  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    //Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
   // while(1);
  }
  drv.begin();

  drv.setMode(DRV2605_MODE_REALTIME);
  
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
}

uint32_t ghostColor[] ={ring.Color(0xff,0x00,0x00),ring.Color(0xfe,0x00,0xff),ring.Color(0x00,0xff,0x00),ring.Color(0xf9,0x9c,0x00)};

/*
 * Protocoll
 * receive:
 * 20byte array
 * 0-3: Neofruit segment of angle value 0-11. 0xff if ghost not on map
 * 4-7: Distance from Pacman. Unsigned Byte max value 255. If farer away also 255
 * 
 *Send:
 *String, Semikolon value
 * first: key pressed. same code as up,left,right, down key on MS-keyboard. As Decimal value sent
 * second: speed of PacMan max speed 1. Normal speed 15. The lower the value the faster is PacMan 
 */
void loop(){
  String inputString;
  byte buf[20];
  bool read = false;
  if (Serial.available()) {
    read = true;
    Serial.readBytes(buf,20);
    for(int i = 0; i < 20; ++i){
      inputString += (char) buf[i];  
    }
    for(int i = 0; i < NUM_PIXELS; ++i){
      setGhost(i,buf);
    }
    Serial.println(buf[0]);
    //Serial.println(inputString); 
  }
  
  
    
  int left = digitalRead(leftButton);
  int up = digitalRead(upButton);
  int right = digitalRead(rightButton);
  int down = digitalRead(downButton);
  int value = 0;
  sensors_event_t event; 
  accel.getEvent(&event);
  int x = event.acceleration.x;
  int y = event.acceleration.y;
  int z = event.acceleration.z;
  if(y <-2 )
     value = 39;//Serial.print("R"); 
  if(y>6)
      value = 37;//Serial.print("L"); 
  if(x>0)
     value = 40;//Serial.print("V");
  if(x < -7)
     value = 38;//Serial.print("Z");
  uint8_t minDistance = minArr(buf);
  drv.setRealtimeValue(max(0,0x30-(minDistance/2.5)));
 
  int pacSpeed = getPacmanSpeed(BPM); //heartValue/60+1;
  Serial.println(String(value,DEC)+";"+String(pacSpeed,DEC)); 
  if (value == HIGH) {
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW);
  }
  //delay(100);

  
   delay(20);
}

uint16_t minArr(byte buf[]){
   uint16_t min = 1000;
   for(uint16_t ghost_id = 0; ghost_id < 4;++ghost_id){
    if((uint8_t)buf[ghost_id+4] < min){
      min = buf[ghost_id+4];
    }
  }
  return min;
}


int getPacmanSpeed(int bpm){
  if (bpm > 200) bpm = 200;
  return (200 - bpm)/20 + 8;
}

//sets the pixel of 
void setGhost(uint16_t px, byte buf[]){
  bool set = false;
  for(uint16_t ghost_id = 0; ghost_id < 4;++ghost_id){
    if(buf[ghost_id] == 0xff)
      continue;
    if(buf[ghost_id] == px && ghost_id == argMin((uint8_t) px,buf)){
      setPixel(px,ghostColor[argMin((uint8_t) px,buf)]);
      set = true;
    }
  }
  if(!set)
    setPixel(px,0,0,0);
}

uint16_t argMin(uint8_t px, byte buf[]){
   int64_t min = 1000;
   uint16_t argMin = 20;
   for(uint16_t ghost_id = 0; ghost_id < 4;++ghost_id){
    if((uint8_t)buf[ghost_id+4] < min && (((uint8_t) buf[ghost_id])==px)){
      min = buf[ghost_id+4];
      argMin = ghost_id;
    }
  }
  return argMin;
}

void setPixel(uint16_t pixel, char r, char g, char b){
  uint32_t color = ring.Color(r, g, b);
  setPixel(pixel,color);
}

void setPixel(uint16_t pixel, uint32_t color){
  ring.setPixelColor((pixel+1%12), color);
  ring.show();
}
