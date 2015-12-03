// ****  Horizon artificiel avec MPU 6050    ******
// Le programme a tendance à bloquer au bout d'un certain temps.
// A vérifier  !!!!
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
// Afficheur tactile 2.8 TFT TouchShield V2 ADA 1651.
// MPU 6050 ref. GY 521. Broche INT sur PIN 2 Arduino UNO.


#include "Adafruit_ILI9341.h"
#include "Adafruit_GFX.h"
#include "SPI.h"
// !!!!  Les librairies I2Cdev et MPU6050 doivent être installées !!!!
// !!!!  ainsi que la briairie pour écran TFT !!!!
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

#define OUTPUT_READABLE_YAWPITCHROLL

#define TFT_DC 9
#define TFT_CS 10

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

int axe_x;
int axe_y;
int axe_xm;
int axe_ym;
int mes;
int var_y=120;
int var_ym;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
	
	tft.begin();
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    // =Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    // =Serial.println(F("Testing device connections..."));
    // =Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
	delay(2000);
	/*
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
	*/
    // load and configure the DMP
    // =Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        // =Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // =Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // =Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // =Serial.print(F("DMP Initialization failed (code "));
        // =Serial.print(devStatus);
        // =Serial.println(F(")"));
    }

	tft.setRotation(1);
	tft.fillScreen(ILI9341_WHITE);
	tft.fillRoundRect(50,0,220,240,20,tft.color565(255,255,128));
	tft.drawFastHLine(50,120,220,ILI9341_NAVY);
	tft.setCursor(0,0);
	tft.setTextColor(ILI9341_NAVY);
	tft.setTextSize(2);
	
	for (int count_p = 0; count_p <= 50; count_p += 10)
	{
		tft.setCursor(1, 115 - (count_p*2));
		tft.print(count_p);
	}
	for (int count_p = 10; count_p <= 50; count_p += 10)
	{
		tft.setCursor(1, 115 + (count_p*2));
		tft.print(- count_p);
	}
	
	for (int count_p = 0; count_p <= 50; count_p += 10)
	{
		tft.setCursor(280, 115 - (count_p*2));
		tft.print(count_p);
	}
	for (int count_p = 10; count_p <= 50; count_p += 10)
	{
		tft.setCursor(280, 115 + (count_p*2));
		tft.print(- count_p);
	}		
	
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
	//

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();  // *** ligne en plus pour éviter bug !! ***
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			/*
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
			*/
        #endif
		
		ypr[1] = ((ypr[1] * 180.0/M_PI));         // Roulis
		ypr[2] = ((ypr[2] * 180.0/M_PI)) * 1.36;  // Tangage
		if (ypr[2] < 0 && ypr[2] >= -50)
		{
			var_y = 120 + (ypr[2] * 2);
		}
		else if (ypr[2] >= 0 && ypr[2] <= 50)
		{
			var_y = 120 + (ypr[2] * 2);
		}

		int y = map(ypr[1] , -140, 140, 0, 360);

		axe_x = 160 - cos((y * 3.141593) / 180.0) * 90.0;
		axe_y = var_y - sin((y * 3.141593) / 180.0) * 90.0;
		
		if (axe_xm != axe_x || axe_ym != axe_y || var_ym != var_y)
		{
			tft.drawLine(axe_xm, axe_ym, 320 - axe_xm, (var_ym * 2) - axe_ym, tft.color565(255,255,128));
			//tft.drawCircle(320 - axe_xm, 240 -axe_ym, 4, ILI9341_WHITE);
			
			tft.drawLine(axe_x, axe_y, 320 - axe_x, (var_y * 2) - axe_y, ILI9341_RED);
			//tft.drawCircle(320 - axe_x, 240 -axe_y, 4, ILI9341_RED);
			tft.drawFastHLine(50, 120, 220, ILI9341_NAVY);	
			tft.drawFastVLine(160, 5, 230, ILI9341_NAVY);		
			axe_xm = axe_x;
			axe_ym = axe_y;
			var_ym = var_y;
		}

    }
}
