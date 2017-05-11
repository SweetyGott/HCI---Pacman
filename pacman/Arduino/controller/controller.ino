//Debug Messages in the Serial output
//#define DEBUG

//Using for the Calibration of the Adafruit Ring
//#define ADAFRUIT_CALIBRATION

/*** ##############################
* Adafruit Neopixel Rings*/
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

/*** ##############################
 *  Jeff Rowberg wrote some Arduino libraries to obtain the accelerometer / gyro data and handle all the calculations. They are available as a zip file from here:
 *  https://github.com/jrowberg/i2cdevlib/zipball/master
 *  Once unzipped, find the Arduino folder within it and copy the two folders "I2Cdev" and "MPU6050" over to your Arduino "libraries" folder in the following directory:
 *  C:\Program Files (x86)\Arduino\libraries
 *  I2C and MPU6050*/
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE FOR MPU        ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//Usage of the old accelorometer sensor
/*#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>*/

/*** ##############################
* Haptric COntroller DRV2605L*/
//#include "Adafruit_DRV2605.h"

// ================================================================
// ===               Adafruit Ring                              ===
// ================================================================
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
//Small Ring
#define PIN_SMALL_RING 9
#define SMALL_RING_NUM_PIXELS 12
#define SMALL_RING_OFFSET 8
Adafruit_NeoPixel small_ring = Adafruit_NeoPixel(SMALL_RING_NUM_PIXELS, PIN_SMALL_RING, NEO_GRB + NEO_KHZ800);

//Big Ring
#define PIN_BIG_RING 8
#define BIG_RING_NUM_PIXELS 24
#define BIG_RING_OFFSET 19
Adafruit_NeoPixel big_ring = Adafruit_NeoPixel(BIG_RING_NUM_PIXELS, PIN_BIG_RING, NEO_GRB + NEO_KHZ800);

//Haptic Driver
//Adafruit_DRV2605 haptic_drv;

//#define BAUDERATE 115200
#define BAUDERATE 9600



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
#ifdef ADAFRUIT_CALIBRATION
    int counter = 0;
#else
    /*** set up the Adafruit rings***/
    small_ring.begin();
    small_ring.setBrightness(64);
    small_ring.show(); // Initialize all pixels to 'off'
    big_ring.begin();
    big_ring.setBrightness(64);
    big_ring.show(); // Initialize all pixels to 'off'
    
    /*** join I2C bus (I2Cdev library doesn't do this automatically)***/
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(BAUDERATE);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
  
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    #ifdef DEBUG
        Serial.println(F("Initializing I2C devices..."));
    #endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    #ifdef DEBUG
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    #endif
    // wait for ready
    #ifdef DEBUG
        Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        while (Serial.available() && Serial.read()); // empty buffer
        while (!Serial.available());                 // wait for data
        while (Serial.available() && Serial.read()); // empty buffer again
    #endif

    // load and configure the DMP
    #ifdef DEBUG
        Serial.println(F("Initializing DMP..."));
    #endif
    devStatus = mpu.dmpInitialize();    
    
    //in case another board is used, I think not necessary 
    //sets clock to 16Mhz clock
    #if defined (__AVR_ATtiny85__)
        if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
    #endif

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        #ifdef DEBUG
            Serial.println(F("Enabling DMP..."));
        #endif
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        #ifdef DEBUG
            Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        #endif
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        #ifdef DEBUG
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
        #endif
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    //starting the haptic driver
    //drv.begin();
    //drv.setMode(DRV2605_MODE_REALTIME);
    
#endif 
}

uint32_t ghostColor[] ={small_ring.Color(0xff,0x00,0x00),small_ring.Color(0xfe,0x00,0xff),small_ring.Color(0x00,0xff,0x00),small_ring.Color(0xf9,0x9c,0x00)};


// ================================================================
// ===                      LOOP ROUTINE                        ===
// ================================================================
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
#ifdef ADAFRUIT_CALIBRATION
    uint32_t color_s = small_ring.Color(255, 0, 0);
    setPixel_small(0,color_s);
    uint32_t color_b = big_ring.Color(0, 255, 0);
    setPixel_big(0,color_b);
#else
    String inputString;
    byte buf[20];
    bool read = false;
    
    if (Serial.available()) {
        //read the 20bytes from the games engine
        read = true;
        Serial.readBytes(buf,20);
        for(int i = 0; i < 20; ++i){
          inputString += (char) buf[i];  
        }

        //set the position of the ghosts
        for(int i = 0; i < SMALL_RING_NUM_PIXELS; ++i){
          setGhost(i,buf);
        }
        Serial.println(buf[0]);
        //Serial.println(inputString); 
    }

    //===================================================
    //==             LOOP READING MPU                  ==
    //===================================================
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #ifdef DEBUG
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
        
        int value = 0;
        //set the Key pressed on gyro value
        if( ypr[2] * 180/M_PI < -20 )
         value = 39;//Serial.print("R"); 
        if( ypr[2] * 180/M_PI > 20 )
          value = 37;//Serial.print("L"); 
        if( ypr[1] * 180/M_PI < -20 )
         value = 40;//Serial.print("U");
        if( ypr[1] * 180/M_PI > 20 )
         value = 38;//Serial.print("D");
        
        //Haptic Control
        //uint8_t minDistance = minArr(buf);
        //drv.setRealtimeValue(max(0,0x30-(minDistance/2.5)));
        
        //int pacSpeed = getPacmanSpeed(60);
        //int pacSpeed = getPacmanSpeed(BPM); //heartValue/60+1;
        Serial.println(String(value,DEC)+";"+String(15,DEC)); 
        
        /*if (value == HIGH) {
            digitalWrite(ledPin, HIGH);
        }
        else {
        digitalWrite(ledPin, LOW);
        }*/
        //delay(100);
        
        
        //delay(20);
    }
#endif
}

//Gets the minimal distance of the 4 ghosts array
uint16_t minArr(byte buf[]){
    uint16_t min = 1000;
    for(uint16_t ghost_id = 0; ghost_id < 4;++ghost_id){
        if((uint8_t)buf[ghost_id+4] < min){
            min = buf[ghost_id+4];
        }
    }
    return min;
}

//sets the pixel of the ghohsts
void setGhost(uint16_t px, byte buf[]){
    bool set = false;
    for(uint16_t ghost_id = 0; ghost_id < 4;++ghost_id){
        //The Angle is defined with a value between 0 and 11
        if(buf[ghost_id] == 0xff)
            continue;
        if(buf[ghost_id] == px && ghost_id == argMin((uint8_t) px,buf)){
            setPixel_small(px,ghostColor[argMin((uint8_t) px,buf)]);
            set = true;
        }
    }
    if(!set)
        setPixel_small(px,0,0,0);
}

//Still unclear what it exactly does
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

void setPixel_small(uint16_t pixel, char r, char g, char b){
    uint32_t color = small_ring.Color(r, g, b);
    setPixel_small(pixel,color);
}

void setPixel_small(uint16_t pixel, uint32_t color){
    small_ring.setPixelColor((pixel+1+SMALL_RING_OFFSET)%SMALL_RING_NUM_PIXELS, color);
    small_ring.show();
}

void setPixel_big(uint16_t pixel, char r, char g, char b){
    uint32_t color = big_ring.Color(r, g, b);
    setPixel_big(pixel,color);
}

void setPixel_big(uint16_t pixel, uint32_t color){
    big_ring.setPixelColor((pixel+1+BIG_RING_OFFSET)%BIG_RING_NUM_PIXELS, color);
    big_ring.show();
}
