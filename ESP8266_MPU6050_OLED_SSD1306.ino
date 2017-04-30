// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "I2Cdev.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
#include "MPU6050.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include <Arduino.h> 
// #include <U8g2lib.h> //https://github.com/olikraus/U8g2_Arduino
#include "SSD1306Wire.h" //https://github.com/squix78/esp8266-oled-ssd1306
//#include "SH1106Wire.h" //https://github.com/squix78/esp8266-oled-ssd1306

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
   #include <Wire.h>
#endif


#if defined(ARDUINO_ESP8266_ESP01) //Arduino Uno
  #define LED_PIN 16
  SSD1306Wire display(0x3c, 2, 0);
  //SH1106Wire display(0x3c, 2, 0);

#elif defined(ARDUINO_ESP8266_NODEMCU)
  #define LED_PIN D0
  SSD1306Wire display(0x3c, D2, D1);
  //SH1106Wire display(0x3c, D2, D1);
  
#endif


// MPU control/status vars
MPU6050 mpu;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
char AcXStr[7];
char AcYStr[7];
char AcZStr[7];
char GyXStr[7];
char GyYStr[7];
char GyZStr[7];

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

  display.setColor(BLACK);
  for (int i = 0; i < OldLinestoRender; i++ )
  {
    display.drawLine(ORender[i].p0.x, ORender[i].p0.y, ORender[i].p1.x, ORender[i].p1.y); // erase the old lines.
  }
  display.drawString(19, 0, AcXStr); // write something to the internal memory
  display.drawString(19, 15, AcYStr); // write something to the internal memory
  display.drawString(19, 27, AcZStr); // write something to the internal memory
  display.drawString(19, 39, GyXStr); // write something to the internal memory
  display.drawString(19, 51, GyYStr); // write something to the internal memory
  display.drawString(79, 51, GyZStr); // write something to the internal memory

  display.setColor(WHITE);

  for (int i = 0; i < LinestoRender; i++ )
  {
    display.drawLine(Render[i].p0.x, Render[i].p0.y, Render[i].p1.x, Render[i].p1.y);
  }
  OldLinestoRender = LinestoRender;

  sprintf (AcXStr, "%6d", AcX);
  sprintf (AcYStr, "%6d", AcY);
  sprintf (AcZStr, "%6d", AcZ);
  sprintf (GyXStr, "%6d", GyX);
  sprintf (GyYStr, "%6d", GyY);
  sprintf (GyZStr, "%6d", GyZ);

  display.drawString(19, 0, AcXStr); // write something to the internal memory
  display.drawString(19, 15, AcYStr); // write something to the internal memory
  display.drawString(19, 27, AcZStr); // write something to the internal memory
  display.drawString(19, 39, GyXStr); // write something to the internal memory
  display.drawString(19, 51, GyYStr); // write something to the internal memory
  display.drawString(79, 51, GyZStr); // write something to the internal memory
  display.display();

}

void setup() {

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

  #if defined(__AVR_ATmega328P__) //Arduino Uno
    Wire.begin();
  #elif defined(ARDUINO_ESP8266_NODEMCU) 
    Wire.begin(SDA, SCL);
    Wire.setClock(400000);
  #elif defined(ARDUINO_ESP8266_ESP01)
    Wire.begin(2, 0);
    Wire.setClock(400000);
  #endif

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  display.init();
  display.clear();
  display.setContrast(32);

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

  display.flipScreenVertically();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  display.drawString(0, 0, "Xa:"); // write something to the internal memory
  display.drawString(0, 15, "Ya:"); // write something to the internal memory
  display.drawString(0, 27, "Za:"); // write something to the internal memory
  display.drawString(0, 39, "Xg:"); // write something to the internal memory
  display.drawString(0, 51, "Yg:"); // write something to the internal memory
  display.drawString(60, 51, "Zg:"); // write something to the internal memory
  display.display();

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

