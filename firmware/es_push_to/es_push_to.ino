#include <TimerOne.h>
#include <TM74HC595Display.h>

#define SCLK  6 // TM74HC595 Pin
#define RCLK  5 // TM74HC595 Pin
#define DIO   4 // TM74HC595 Pin

#define FIRST_DIGIT   3
#define SECOND_DIGIT  2
#define THIRD_DIGIT   1
#define FOURTH_DIGIT  0
#define POINT_SHIFT   10

#define EMPTY_SYMBOL  0b11111111 // " "
#define DASH_SYMBOL   0b10111111 // "-"


#define AZIMUTH_ENCODER_PIN_A 2
#define AZIMUTH_ENCODER_PIN_B 3
#define AZIMUTH_ENCODER_RESOLUTION 600 // 600 * 2

TM74HC595Display disp(SCLK, RCLK, DIO);

unsigned char SYMBOL[20];

volatile int counter = 0; // This variable will increase or decrease depending on the rotation of encoder
int current = 0;

void setup() {
  Serial.begin (9600);

  initAzimuthEncoder();
  initDisplay();
}

void initAzimuthEncoder() {
  pinMode(AZIMUTH_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(AZIMUTH_ENCODER_PIN_B, INPUT_PULLUP);

  // Setting up interrupt
  // A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);

  // B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
}

void initDisplay() {
  symbols();

  Timer1.initialize(1500);
  Timer1.attachInterrupt(timerIsr);
}

void loop() {
  if (current != counter) {
    current = counter;
    Serial.println (counter);

    //displayAzimuth(getAzimuth());
  }

  displayAzimuth(counter);
}

float getAzimuth() {
  return (360.0 / AZIMUTH_ENCODER_RESOLUTION) * counter;
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW) {
    rotateRight();
  } else {
    rotateLeft();
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    rotateLeft();
  } else {
    rotateRight();
  }
}

void rotateRight() {
  counter++;
  if (counter > AZIMUTH_ENCODER_RESOLUTION) {
    //counter = 0;
  }
}

void rotateLeft() {
  counter--;
  if (counter < 0) {
    //counter = AZIMUTH_ENCODER_RESOLUTION;
  }
}

void timerIsr() {   // прерывание таймера
  disp.timerIsr();  // пнуть дисплей
}

void symbols() {
  // обычные
  SYMBOL[0] = 0xC0; //0
  SYMBOL[1] = 0xF9; //1
  SYMBOL[2] = 0xA4; //2
  SYMBOL[3] = 0xB0; //3
  SYMBOL[4] = 0x99; //4
  SYMBOL[5] = 0x92; //5
  SYMBOL[6] = 0x82; //6
  SYMBOL[7] = 0xF8; //7
  SYMBOL[8] = 0x80; //8
  SYMBOL[9] = 0x90; //9

  // с точкой
  SYMBOL[10] = 0b01000000; //0.
  SYMBOL[11] = 0b01111001; //1.
  SYMBOL[12] = 0b00100100; //2.
  SYMBOL[13] = 0b00110000; //3.
  SYMBOL[14] = 0b00011001; //4.
  SYMBOL[15] = 0b00010010; //5.
  SYMBOL[16] = 0b00000010; //6.
  SYMBOL[17] = 0b01111000; //7.
  SYMBOL[18] = 0b00000000; //8.
  SYMBOL[19] = 0b00010000; //9.
}

void displayAzimuth(float azimuth) {
  int firstDigit = (int) azimuth / 100; // 768 -> 7
  int secondDigit =  (int)(azimuth - firstDigit * 100) / 10; // 768 -> 6
  int thirdDigit = (int) azimuth % 10;  // 768 -> 8

  int fractionDigit = 0;

  disp.set((firstDigit > 0) ? SYMBOL[firstDigit] : EMPTY_SYMBOL, FIRST_DIGIT);
  disp.set((secondDigit > 0) ? SYMBOL[secondDigit] : EMPTY_SYMBOL, SECOND_DIGIT);
  disp.set(SYMBOL[POINT_SHIFT + thirdDigit], THIRD_DIGIT);
  disp.set(SYMBOL[fractionDigit], FOURTH_DIGIT);
}

void displayTest() {
  disp.set(SYMBOL[3], FIRST_DIGIT);
  disp.set(SYMBOL[5], SECOND_DIGIT);
  disp.set(SYMBOL[16], THIRD_DIGIT);
  disp.set(SYMBOL[8], FOURTH_DIGIT);
}
