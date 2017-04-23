// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "I2Cdev.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
#include "MPU6050.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include <Arduino.h> 
#include <U8g2lib.h> //https://github.com/olikraus/U8g2_Arduino

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
   #include <Wire.h>
#endif


#if defined(__AVR_ATmega328P__) //Arduino Uno
  #define LED_PIN 13
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
  //U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#elif defined(ARDUINO_ESP8266_NODEMCU)
  #define LED_PIN D0
  //U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* reset=*/ 8);
  U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=D1(SCL)*/ SCL, /* data=D2(SDA)*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
  //U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=D1(SCL)*/ SCL, /* data=D2(SDA)*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display

#endif


// MPU control/status vars
MPU6050 mpu;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

bool blinkState = false;


/* MrAbalogu/Smart-Watch/MPU6050_3D_Cube */
/* Reference from https://github.com/MrAbalogu/Smart-Watch/blob/master/MPU6050_3D_Cube/MPU6050_3D_Cube.ino */

float xx, xy, xz;
float yx, yy, yz;
float zx, zy, zz;

float fact;

int Xan, Yan;

int Xoff;
int Yoff;
int Zoff;

struct Point3d
{
  int x;
  int y;
  int z;
};

struct Point2d
{
  int x;
  int y;
};

int LinestoRender; // lines to render.
int OldLinestoRender; // lines to render just in case it changes. this makes sure the old lines all get erased.

struct Line3d
{
  Point3d p0;
  Point3d p1;
};

struct Line2d
{
  Point2d p0;
  Point2d p1;
};

Line3d Lines[12];  //Number of lines to render
Line2d Render[12];
Line2d ORender[12];

/************ MrAbalogu/Smart-Watch/MPU6050_3D_Cube*********************************************************************/

void initLine() {
  fact = 180 / 3.14159265358979323846264338327950; // conversion from degrees to radians.

  Xoff = 90; // positions the center of the 3d conversion space into the center of the OLED screen. This is usally screen_x_size / 2.
  Yoff = 32; // screen_y_size /2
  Zoff = 750;   //Size of cube, larger no. = smaller cube

  // line segments to draw a cube. basically p0 to p1. p1 to p2. p2 to p3 so on.

  // Front Face.

  Lines[0].p0.x = -50;
  Lines[0].p0.y = -50;
  Lines[0].p0.z = 50;
  Lines[0].p1.x = 50;
  Lines[0].p1.y = -50;
  Lines[0].p1.z = 50;

  Lines[1].p0.x = 50;
  Lines[1].p0.y = -50;
  Lines[1].p0.z = 50;
  Lines[1].p1.x = 50;
  Lines[1].p1.y = 50;
  Lines[1].p1.z = 50;

  Lines[2].p0.x = 50;
  Lines[2].p0.y = 50;
  Lines[2].p0.z = 50;
  Lines[2].p1.x = -50;
  Lines[2].p1.y = 50;
  Lines[2].p1.z = 50;

  Lines[3].p0.x = -50;
  Lines[3].p0.y = 50;
  Lines[3].p0.z = 50;
  Lines[3].p1.x = -50;
  Lines[3].p1.y = -50;
  Lines[3].p1.z = 50;


  //back face.

  Lines[4].p0.x = -50;
  Lines[4].p0.y = -50;
  Lines[4].p0.z = -50;
  Lines[4].p1.x = 50;
  Lines[4].p1.y = -50;
  Lines[4].p1.z = -50;

  Lines[5].p0.x = 50;
  Lines[5].p0.y = -50;
  Lines[5].p0.z = -50;
  Lines[5].p1.x = 50;
  Lines[5].p1.y = 50;
  Lines[5].p1.z = -50;

  Lines[6].p0.x = 50;
  Lines[6].p0.y = 50;
  Lines[6].p0.z = -50;
  Lines[6].p1.x = -50;
  Lines[6].p1.y = 50;
  Lines[6].p1.z = -50;

  Lines[7].p0.x = -50;
  Lines[7].p0.y = 50;
  Lines[7].p0.z = -50;
  Lines[7].p1.x = -50;
  Lines[7].p1.y = -50;
  Lines[7].p1.z = -50;

  // now the 4 edge lines.

  Lines[8].p0.x = -50;
  Lines[8].p0.y = -50;
  Lines[8].p0.z = 50;
  Lines[8].p1.x = -50;
  Lines[8].p1.y = -50;
  Lines[8].p1.z = -50;

  Lines[9].p0.x = 50;
  Lines[9].p0.y = -50;
  Lines[9].p0.z = 50;
  Lines[9].p1.x = 50;
  Lines[9].p1.y = -50;
  Lines[9].p1.z = -50;

  Lines[10].p0.x = -50;
  Lines[10].p0.y = 50;
  Lines[10].p0.z = 50;
  Lines[10].p1.x = -50;
  Lines[10].p1.y = 50;
  Lines[10].p1.z = -50;

  Lines[11].p0.x = 50;
  Lines[11].p0.y = 50;
  Lines[11].p0.z = 50;
  Lines[11].p1.x = 50;
  Lines[11].p1.y = 50;
  Lines[11].p1.z = -50;

  LinestoRender = 12;
  OldLinestoRender = LinestoRender;
}

/************ MrAbalogu/Smart-Watch/MPU6050_3D_Cube*********************************************************************/
// Sets the global vars for the 3d transform. Any points sent through "process" will be transformed using these figures.
// only needs to be called if Xan or Yan are changed.
void SetVars(void)
{
  float Xan2, Yan2, Zan2;
  float s1, s2, s3, c1, c2, c3;

  Xan2 = Xan / fact; // convert degrees to radians.
  Yan2 = Yan / fact;

  // Zan is assumed to be zero

  s1 = sin(Yan2);
  s2 = sin(Xan2);

  c1 = cos(Yan2);
  c2 = cos(Xan2);

  xx = c1;
  xy = 0;
  xz = -s1;

  yx = (s1 * s2);
  yy = c2;
  yz = (c1 * s2);

  zx = (s1 * c2);
  zy = -s2;
  zz = (c1 * c2);
}


/************ MrAbalogu/Smart-Watch/MPU6050_3D_Cube*********************************************************************/
// processes x1,y1,z1 and returns rx1,ry1 transformed by the variables set in SetVars()
// fairly heavy on floating point here.
// uses a bunch of global vars. Could be rewritten with a struct but not worth the effort.
void ProcessLine(struct Line2d *ret, struct Line3d vec)
{
  float zvt1;
  int xv1, yv1, zv1;

  float zvt2;
  int xv2, yv2, zv2;

  int rx1, ry1;
  int rx2, ry2;

  int x1;
  int y1;
  int z1;

  int x2;
  int y2;
  int z2;

  int Ok;

  x1 = vec.p0.x;
  y1 = vec.p0.y;
  z1 = vec.p0.z;

  x2 = vec.p1.x;
  y2 = vec.p1.y;
  z2 = vec.p1.z;

  Ok = 0; // defaults to not OK

  xv1 = (x1 * xx) + (y1 * xy) + (z1 * xz);
  yv1 = (x1 * yx) + (y1 * yy) + (z1 * yz);
  zv1 = (x1 * zx) + (y1 * zy) + (z1 * zz);

  zvt1 = zv1 - Zoff;


  if ( zvt1 < -5) {
    rx1 = 256 * (xv1 / zvt1) + Xoff;
    ry1 = 256 * (yv1 / zvt1) + Yoff;
    Ok = 1; // ok we are alright for point 1.
  }


  xv2 = (x2 * xx) + (y2 * xy) + (z2 * xz);
  yv2 = (x2 * yx) + (y2 * yy) + (z2 * yz);
  zv2 = (x2 * zx) + (y2 * zy) + (z2 * zz);

  zvt2 = zv2 - Zoff;


  if ( zvt2 < -5) {
    rx2 = 256 * (xv2 / zvt2) + Xoff;
    ry2 = 256 * (yv2 / zvt2) + Yoff;
  } else
  {
    Ok = 0;
  }

  if (Ok == 1) {
    ret->p0.x = rx1;
    ret->p0.y = ry1;

    ret->p1.x = rx2;
    ret->p1.y = ry2;
  }
  // The ifs here are checks for out of bounds. needs a bit more code here to "safe" lines that will be way out of whack, so they dont get drawn and cause screen garbage.

}

/************ MrAbalogu/Smart-Watch/MPU6050_3D_Cube*********************************************************************/
void calculateCube() {
  //For cube rotation
  int xOut = 0;
  int yOut = 0;

  xOut = map(AcX, -17000, 17000, -50, 50);
  yOut = map(AcY, -17000, 17000, -50, 50);

  Xan += xOut;
  Yan += yOut;


  Yan = Yan % 360;
  Xan = Xan % 360; // prevents overflow.



  SetVars(); //sets up the global vars to do the conversion.

  for (int i = 0; i < LinestoRender ; i++)
  {
    ORender[i] = Render[i]; // stores the old line segment so we can delete it later.
    ProcessLine(&Render[i], Lines[i]); // converts the 3d line segments to 2d.
  }
}

void drawScreen( void)
{
  // renders all the lines after erasing the old ones.
  // in here is the only code actually interfacing with the OLED. so if you use a different lib, this is where to change it.

  u8g2.setDrawColor(0);
  for (int i = 0; i < OldLinestoRender; i++ )
  {
    u8g2.drawLine(ORender[i].p0.x, ORender[i].p0.y, ORender[i].p1.x, ORender[i].p1.y); // erase the old lines.
  }

  u8g2.setDrawColor(1);


  for (int i = 0; i < LinestoRender; i++ )
  {
    u8g2.drawLine(Render[i].p0.x, Render[i].p0.y, Render[i].p1.x, Render[i].p1.y);
  }
  OldLinestoRender = LinestoRender;
  char AcXStr[7];
  sprintf (AcXStr, "%6d", AcX);

  char AcYStr[7];
  sprintf (AcYStr, "%6d", AcY);

  char AcZStr[7];
  sprintf (AcZStr, "%6d", AcZ);

  char GyXStr[7];
  sprintf (GyXStr, "%6d", GyX);

  char GyYStr[7];
  sprintf (GyYStr, "%6d", GyY);

  char GyZStr[7];
  sprintf (GyZStr, "%6d", GyZ);

  u8g2.drawStr(19, 15, AcXStr); // write something to the internal memory
  u8g2.drawStr(19, 27, AcYStr); // write something to the internal memory
  u8g2.drawStr(19, 39, AcZStr); // write something to the internal memory
  u8g2.drawStr(19, 51, GyXStr); // write something to the internal memory
  u8g2.drawStr(19, 63, GyYStr); // write something to the internal memory
  u8g2.drawStr(79, 63, GyZStr); // write something to the internal memory
  u8g2.sendBuffer();

}

void setup() {

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

  #if defined(__AVR_ATmega328P__) //Arduino Uno
    Wire.begin();
  #elif defined(ARDUINO_ESP8266_NODEMCU)
    Wire.begin(SDA, SCL);
    Wire.setClock(400000);
  #endif

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  u8g2.begin();

  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Serial.println("Updating internal sensor offsets...");
  Serial.print(mpu.getXAccelOffset()); Serial.print("\t"); //
  Serial.print(mpu.getYAccelOffset()); Serial.print("\t"); //
  Serial.print(mpu.getZAccelOffset()); Serial.print("\t"); //
  Serial.print(mpu.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(mpu.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(mpu.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");

  mpu.setXAccelOffset(-1782);
  mpu.setYAccelOffset(-889);
  mpu.setZAccelOffset(1427);
  mpu.setXGyroOffset (11);
  mpu.setYGyroOffset (-5);
  mpu.setZGyroOffset (23);

  Serial.print(mpu.getXAccelOffset()); Serial.print("\t"); //
  Serial.print(mpu.getYAccelOffset()); Serial.print("\t"); //
  Serial.print(mpu.getZAccelOffset()); Serial.print("\t"); //
  Serial.print(mpu.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(mpu.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(mpu.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");


  initLine();

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  u8g2.setFontMode(0);
  u8g2.setFont(u8g2_font_6x10_mf);

  u8g2.clearBuffer();

  u8g2.drawStr(0, 15, "Xa:"); // write something to the internal memory
  u8g2.drawStr(0, 27, "Ya:"); // write something to the internal memory
  u8g2.drawStr(0, 39, "Za:"); // write something to the internal memory
  u8g2.drawStr(0, 51, "Xg:"); // write something to the internal memory
  u8g2.drawStr(0, 63, "Yg:"); // write something to the internal memory
  u8g2.drawStr(60, 63, "Zg:"); // write something to the internal memory

}

void loop() {
  
    /* read raw accel/gyro measurements from device without DMP6 FIFO 
     AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; */
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    /*these methods (and a few others) are also available (without DMP6 FIFO) */
    // mpu.getAcceleration(&AcX, &AcY, &AcZ);
    // mpu.getRotation(&GyX, &GyY, &GyZ);
    Tmp = mpu.getTemperature();

    calculateCube();

    //OLED : u8g2 draw
    drawScreen(); 
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

}

