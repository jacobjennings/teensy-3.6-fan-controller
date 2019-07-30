#include <SPI.h>
#include <FreqMeasureMulti.h>
#include <i2c_t3.h>

// Pin selections

// These SPI pins 11 and 12 don't have PWM etc, so we aren't wasting PWM pins.
static uint8_t kPin_SPI0_MOSI = 11;
static uint8_t kPin_SPI0_MISO = 12;

// Using 27 instead of 13 leaves 13 open for freq counting
static uint8_t kPin_SPI0_SCK = 27; 

// 15 is a CS0 pin which is not also PWM.
static uint8_t kPin_SPI0_CS = 15;

// Let's reserve as much i2s as possible so we can run multiple chips
// with the same addresses. Accelerometer module I have can change its
// address, so 2 addresses per chip * 4 i2s buses = 8 same-chips

// I2S0: Non-pwm pin selections
static uint8_t kPin_I2S0_SCL = 19;
static uint8_t kPin_I2S0_SDA = 18;

// All I2S1 pins overlap with PWM. 
static uint8_t kPin_I2S1_SCL = 37;
static uint8_t kPin_I2S1_SDA = 38;

// I2S2 - one PWM stomped by SDA.
static uint8_t kPin_I2S2_SCL = 26;
static uint8_t kPin_I2S2_SDA = 4;

// I2S3 on the back side doesn't overlap with PWM.
static uint8_t kPin_I2S3_SCL = 57;
static uint8_t kPin_I2S3_SDA = 56;

// Remaining 12 PWM pins:
static uint8_t kPin_PWM0 = 2;
static uint8_t kPin_PWM1 = 3;
static uint8_t kPin_PWM2 = 7;
static uint8_t kPin_PWM3 = 8;
static uint8_t kPin_PWM4 = 29;
static uint8_t kPin_PWM5 = 30;
static uint8_t kPin_PWM6 = 17;
static uint8_t kPin_PWM7 = 16;
static uint8_t kPin_PWM8 = 14;
static uint8_t kPin_PWM9 = 36;
static uint8_t kPin_PWM10 = 35;

// General use digital I/O in order of least interesting first (avoid reserving pins that do other things)
static uint8_t kPin_DIO0 = 24;
static uint8_t kPin_DIO1 = 25;
static uint8_t kPin_DIO2 = 28;
static uint8_t kPin_DIO3 = 31;
static uint8_t kPin_DIO4 = 32;
static uint8_t kPin_DIO5 = 0; // rx1
static uint8_t kPin_DIO6 = 1; // tx1
static uint8_t kPin_DIO7 = 39;
static uint8_t kPin_DIO8 = 34;
static uint8_t kPin_DIO9 = 33;
// Back side
static uint8_t kPin_DIO10 = 40;
static uint8_t kPin_DIO11 = 41;
static uint8_t kPin_DIO12 = 42;
static uint8_t kPin_DIO13 = 43;
static uint8_t kPin_DIO14 = 44;
static uint8_t kPin_DIO15 = 45;
static uint8_t kPin_DIO16 = 46;
static uint8_t kPin_DIO17 = 47;
static uint8_t kPin_DIO18 = 48;
static uint8_t kPin_DIO19 = 49;
static uint8_t kPin_DIO20 = 50;
static uint8_t kPin_DIO21 = 51;
static uint8_t kPin_DIO22 = 52;
static uint8_t kPin_DIO23 = 53;
static uint8_t kPin_DIO24 = 54;
static uint8_t kPin_DIO25 = 55;

// Pin 13 is special, leave it alone.
static uint8_t kPin_Special13 = 13;


// FreqMeasureMulti library pins.
static uint8_t kPin_FreqMeasureMulti0 = 22; // also pwm
static uint8_t kPin_FreqMeasureMulti1 = 23; // also pwm
static uint8_t kPin_FreqMeasureMulti2 = 9; // also pwm
static uint8_t kPin_FreqMeasureMulti3 = 10; // also pwm
static uint8_t kPin_FreqMeasureMulti4 = 6; // also pwm
static uint8_t kPin_FreqMeasureMulti5 = 20; // also pwm
static uint8_t kPin_FreqMeasureMulti6 = 21; // also pwm
static uint8_t kPin_FreqMeasureMulti7 = 5; // also pwm


// Remaining 12 PWM pins:
static uint8_t kPin_PWM0 = 2;
static uint8_t kPin_PWM1 = 3;
static uint8_t kPin_PWM2 = 7;
static uint8_t kPin_PWM3 = 8;
static uint8_t kPin_PWM4 = 29;
static uint8_t kPin_PWM5 = 30;
static uint8_t kPin_PWM6 = 17;
static uint8_t kPin_PWM7 = 16;
static uint8_t kPin_PWM8 = 14;
static uint8_t kPin_PWM9 = 36;
static uint8_t kPin_PWM10 = 35;

// 2, 22
// 3, 23
// 7, 9
// 8, 10
// 29, 6
// 30, 20
// 17, 21
// 16, 5
// 14, 24
// 36, 25
static uint8_t kNumberOfHalfRevolutionsToAverage = 35;

FreqMeasureMulti gFreqMeasure0;
double gFreqMeasureSum0;
int gFreqMeasureCount0;
float gFreqMeasureRPM0;

FreqMeasureMulti gFreqMeasure1;
double gFreqMeasureSum1;
int gFreqMeasureCount1;
float gFreqMeasureRPM1;

FreqMeasureMulti gFreqMeasure2;
double gFreqMeasureSum2;
int gFreqMeasureCount2;
float gFreqMeasureRPM2;

FreqMeasureMulti gFreqMeasure3;
double gFreqMeasureSum3;
int gFreqMeasureCount3;
float gFreqMeasureRPM3;

FreqMeasureMulti gFreqMeasure4;
double gFreqMeasureSum4;
int gFreqMeasureCount4;
float gFreqMeasureRPM4;

FreqMeasureMulti gFreqMeasure5;
double gFreqMeasureSum5;
int gFreqMeasureCount5;
float gFreqMeasureRPM5;

FreqMeasureMulti gFreqMeasure6;
double gFreqMeasureSum6;
int gFreqMeasureCount6;
float gFreqMeasureRPM6;

FreqMeasureMulti gFreqMeasure7;
double gFreqMeasureSum7;
int gFreqMeasureCount7;
float gFreqMeasureRPM7;

// Manual sense

static uint16_t kManualSenseWindowMs = 1500;
static float kManualSenseWindowMsFloat = (float)kManualSenseWindowMs / 1000.f;

elapsedMicros gManualSenseElapsedMicrosBetweenPulses0;
uint32_t gManualSenseMicrosSum0;
int gManualSenseCount0;
float gManualSenseRPM0;

elapsedMicros gManualSenseElapsedMicrosBetweenPulses1;
uint32_t gManualSenseMicrosSum1;
int gManualSenseCount1;
float gManualSenseRPM1;

void manualSensePulse0() {
  gManualSenseMicrosSum0 += gManualSenseElapsedMicrosBetweenPulses0;
  gManualSenseCount0 = gManualSenseCount0 + 1;
  gManualSenseElapsedMicrosBetweenPulses0 = 0;
}

void manualSensePulse1() {
  gManualSenseMicrosSum1 += gManualSenseElapsedMicrosBetweenPulses1;
  gManualSenseCount1 = gManualSenseCount1 + 1;
  gManualSenseElapsedMicrosBetweenPulses1 = 0;
}

void setup() {
  SPI.setMOSI(kPin_SPI0_MOSI);
  SPI.setMISO(kPin_SPI0_MISO);
  SPI.setSCK(kPin_SPI0_SCK);

  Serial.begin(57600);

  gFreqMeasure0.begin(kPin_FreqMeasureMulti0);    
  gFreqMeasure1.begin(kPin_FreqMeasureMulti1);
  gFreqMeasure2.begin(kPin_FreqMeasureMulti2);
  gFreqMeasure3.begin(kPin_FreqMeasureMulti3);
  gFreqMeasure4.begin(kPin_FreqMeasureMulti4);
  gFreqMeasure5.begin(kPin_FreqMeasureMulti5);
  gFreqMeasure6.begin(kPin_FreqMeasureMulti6);
  gFreqMeasure7.begin(kPin_FreqMeasureMulti7);

  pinMode(kPin_PWM0, OUTPUT);
  pinMode(kPin_PWM1, OUTPUT);
  pinMode(kPin_PWM2, OUTPUT);
  pinMode(kPin_PWM3, OUTPUT);
  pinMode(kPin_PWM4, OUTPUT);
  pinMode(kPin_PWM5, OUTPUT);
  pinMode(kPin_PWM6, OUTPUT);
  pinMode(kPin_PWM7, OUTPUT);
  pinMode(kPin_PWM8, OUTPUT);
  pinMode(kPin_PWM9, OUTPUT);

  // todo: reconfigure according to freqs
  analogWrite(kPin_PWM0, 170);
  analogWrite(kPin_PWM1, 170);
  analogWrite(kPin_PWM2, 170);
  analogWrite(kPin_PWM3, 170);
  analogWrite(kPin_PWM4, 170);
  analogWrite(kPin_PWM5, 170);
  analogWrite(kPin_PWM6, 170);
  analogWrite(kPin_PWM7, 170);
  analogWrite(kPin_PWM8, 170);
  analogWrite(kPin_PWM9, 170);

  pinMode(kPin_DIO0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kPin_DIO0), manualSensePulse0, RISING);
  configureHighPriorityInterruptForPin(kPin_DIO0);

  pinMode(kPin_DIO1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kPin_DIO1), manualSensePulse1, RISING);
  configureHighPriorityInterruptForPin(kPin_DIO1);
}

void loop() {
  processFreqMeasure(&gFreqMeasure0, &gFreqMeasureSum0, &gFreqMeasureCount0, &gFreqMeasureRPM0, 0);
  processFreqMeasure(&gFreqMeasure1, &gFreqMeasureSum1, &gFreqMeasureCount1, &gFreqMeasureRPM1, 1);
  processFreqMeasure(&gFreqMeasure2, &gFreqMeasureSum2, &gFreqMeasureCount2, &gFreqMeasureRPM2, 2);
  processFreqMeasure(&gFreqMeasure3, &gFreqMeasureSum3, &gFreqMeasureCount3, &gFreqMeasureRPM3, 3);
  processFreqMeasure(&gFreqMeasure4, &gFreqMeasureSum4, &gFreqMeasureCount4, &gFreqMeasureRPM4, 4);
  processFreqMeasure(&gFreqMeasure5, &gFreqMeasureSum5, &gFreqMeasureCount5, &gFreqMeasureRPM5, 5);
  processFreqMeasure(&gFreqMeasure6, &gFreqMeasureSum6, &gFreqMeasureCount6, &gFreqMeasureRPM6, 6);
  processFreqMeasure(&gFreqMeasure7, &gFreqMeasureSum7, &gFreqMeasureCount7, &gFreqMeasureRPM7, 7);

  processManualSense(&gManualSenseMicrosSum0, &gManualSenseCount0, &gManualSenseRPM0, 8);
  processManualSense(&gManualSenseMicrosSum1, &gManualSenseCount1, &gManualSenseRPM1, 9);
}

void processFreqMeasure(FreqMeasureMulti *freqMeasureMulti, double *sum, int *count, float *rpm, uint8_t fanNumber) {
  if (freqMeasureMulti->available()) {
    // average several reading together
    *sum = *sum + freqMeasureMulti->read();
    *count = *count + 1;
//    Serial.print(".");
    if (*count > kNumberOfHalfRevolutionsToAverage) {
      *rpm = freqMeasureMulti->countToFrequency(*sum / *count) * 30.f; // Hz * 60 seconds / 2 pulses per revolution
      Serial.print(fanNumber);
      Serial.print(": ");
      Serial.print(*rpm);
      Serial.println(" RPM");
      *sum = 0;
      *count = 0;
    }
  }
}

void processManualSense(uint32_t *microsSum, int *count, float *rpm, uint8_t fanNumber) {
  if (*count > kNumberOfHalfRevolutionsToAverage) {
    float averageIntervalSeconds = (float)*microsSum / 1000000.f / (float)*count;
    *rpm = 1.f / averageIntervalSeconds * 30.f; // Hz * 60 seconds / 2 pulses per revolution
    Serial.print(fanNumber);
    Serial.print(": ");
    Serial.print(*rpm);
    Serial.println(" RPM");
    *count = 0;
    *microsSum = 0;
  }
}

void configureHighPriorityInterruptForPin(int pin) {
  volatile uint32_t *pinConfig;
  pinConfig = portConfigRegister(pin);  

  int irqPort = 0;
  if(&PORTA_PCR0 <= pinConfig && pinConfig <= &PORTA_PCR31) irqPort = IRQ_PORTA;
  else if(&PORTB_PCR0 <= pinConfig && pinConfig <= &PORTB_PCR31) irqPort = IRQ_PORTB;
  else if(&PORTC_PCR0 <= pinConfig && pinConfig <= &PORTC_PCR31) irqPort = IRQ_PORTC;
  else if(&PORTD_PCR0 <= pinConfig && pinConfig <= &PORTD_PCR31) irqPort = IRQ_PORTD;
  else if(&PORTE_PCR0 <= pinConfig && pinConfig <= &PORTE_PCR31) irqPort = IRQ_PORTE;
  
  NVIC_SET_PRIORITY(irqPort, 0);
}
