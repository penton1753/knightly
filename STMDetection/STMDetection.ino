#include <Wire.h>
#include <CD74HC4067.h>

#define IGNORE_FAULTY true

#define IS_BOARD_1 false
#define REFERENCE 2275
#define BLACK_THRESHOLD 275
#define WHITE_THRESHOLD 110

#define DEBUG false

#define EMPTY 0
#define WHITE 1
#define BLACK 2

// Board Selection Multiplexer
CD74HC4067 muxL(PB13, PB12, PB14, PB15);
CD74HC4067 muxR(PA5, PA4, PA6, PA7);

// Sensor Selection Multiplexer
CD74HC4067 rib(PB10, PB_2, PB1, PB0);

const int signalPinL = PA1;
const int signalPinR = PA0;

// Constants for the 0.2V - 3.3V range
const int ADC_MIN = 0;     // Lower bound (0.2V)
const int ADC_MAX = 4095;  // Upper bound (3.3V)

int valArray[12][16];

void fillBuf(int buf[12][16]) {
#if IS_BOARD_1
  for (byte board = 0; board < 6; ++board) {
#else
  for (byte board = 6; board < 12; ++board) {
#endif
    muxL.channel(board);

    // Give the muxL some time to physically settle the connection
    delayMicroseconds(10);

    for (byte sensor = 0; sensor < 16; ++sensor) {
      rib.channel(sensor);
      delayMicroseconds(10);

      // Double read: First read clears the Sample & Hold capacitor
      analogRead(signalPinL);
      int raw = analogRead(signalPinL);
#if IS_BOARD_1
      valArray[board][sensor] = map(constrain(raw, ADC_MIN, ADC_MAX), ADC_MIN, ADC_MAX, 0, 4095);
#else
      valArray[board - 6][sensor] = map(constrain(raw, ADC_MIN, ADC_MAX), ADC_MIN, ADC_MAX, 0, 4095);
#endif
    }
  }

#if IS_BOARD_1
  for (byte board = 0; board < 6; ++board) {
#else
  for (byte board = 6; board < 12; ++board) {
#endif
    muxR.channel(board);

    // Give the muxR some time to physically settle the connection
    delayMicroseconds(10);

    for (byte sensor = 0; sensor < 16; ++sensor) {
      rib.channel(sensor);
      delayMicroseconds(10);

      // Double read: First read clears the Sample & Hold capacitor
      analogRead(signalPinR);
      int raw = analogRead(signalPinR);
#if IS_BOARD_1
      valArray[board + 6][sensor] = map(constrain(raw, ADC_MIN, ADC_MAX), ADC_MIN, ADC_MAX, 0, 4095);
#else
      valArray[board][sensor] = map(constrain(raw, ADC_MIN, ADC_MAX), ADC_MIN, ADC_MAX, 0, 4095);
#endif
    }
  }
}

void sendBinaryState(int buf[12][16]) {
  uint8_t state[96];

  for (int i = 0; i < 96; ++i) state[i] = 0;  // rasters rank then file

#if IS_BOARD_1  // Files E-j, L Ranks 1-4, R Ranks 5-8
  for (int board = 0; board < 12; ++board) {
    for (int sensor = 0; sensor < 16; ++sensor) {
      int file = 11 - board;
      if (board >= 6) file = 17 - board;

      int rank;
      if (sensor == 6 || sensor == 7 || sensor == 8 || sensor == 9) rank = 3;
      if (sensor == 4 || sensor == 5 || sensor == 10 || sensor == 11) rank = 2;
      if (sensor == 2 || sensor == 3 || sensor == 12 || sensor == 13) rank = 1;
      if (sensor == 0 || sensor == 1 || sensor == 14 || sensor == 15) rank = 0;

      if (board >= 6) rank = 7 - rank;

#if IGNORE_FAULTY
      // For the one really weird sensor
      if (board == 9 && sensor == 11) continue;
#endif

      if (state[file * 8 + rank] < WHITE && ((buf[board][sensor] - REFERENCE) < -WHITE_THRESHOLD || (buf[board][sensor] - REFERENCE) > WHITE_THRESHOLD))
        state[file * 8 + rank] = WHITE;
      if (state[file * 8 + rank] < BLACK && ((buf[board][sensor] - REFERENCE) < -BLACK_THRESHOLD || (buf[board][sensor] - REFERENCE) > BLACK_THRESHOLD))
        state[file * 8 + rank] = BLACK;
    }
  }
#else  // Files y-D, L Ranks 5-8, R Ranks 1-4
  for (int board = 0; board < 12; ++board) {
    for (int sensor = 0; sensor < 16; ++sensor) {
      int file = 5 - board;
      if (board >= 6) file = 11 - board;

      int rank;

      if (sensor == 6 || sensor == 7 || sensor == 8 || sensor == 9) rank = 3;
      if (sensor == 4 || sensor == 5 || sensor == 10 || sensor == 11) rank = 2;
      if (sensor == 2 || sensor == 3 || sensor == 12 || sensor == 13) rank = 1;
      if (sensor == 0 || sensor == 1 || sensor == 14 || sensor == 15) rank = 0;

      if (board < 6) rank = 7 - rank;

      if (state[file * 8 + rank] < WHITE && ((buf[board][sensor] - REFERENCE) < -WHITE_THRESHOLD || (buf[board][sensor] - REFERENCE) > WHITE_THRESHOLD))
        state[file * 8 + rank] = WHITE;
      if (state[file * 8 + rank] < BLACK && ((buf[board][sensor] - REFERENCE) < -BLACK_THRESHOLD || (buf[board][sensor] - REFERENCE) > BLACK_THRESHOLD))
        state[file * 8 + rank] = BLACK;
    }
  }
#endif

  Serial.write('T');
  for (int i = 0; i < 96; ++i) {
    if (state[i] == BLACK) Serial.write('B');
    else if (state[i] == WHITE) Serial.write('W');
    else Serial.write('_');
  }
  Serial.print("\n");
}

void setup() {
  // STM32 handles high baud rates easily. 115200 is recommended.
  Serial.begin(115200);

  pinMode(signalPinL, INPUT_ANALOG);  // Explicitly set for STM32 Analog
  pinMode(signalPinR, INPUT_ANALOG);  // Explicitly set for STM32 Analog

  // Set ADC Resolution to 12-bit (0-4095)
  analogReadResolution(12);
}

void loop() {
  if (Serial.available() > 0) {     // Check if data is waiting
    byte nextByte = Serial.read();  // Read the first available byte

    if (nextByte == 'T') {
      fillBuf(valArray);
      sendBinaryState(valArray);
    }
  }

  //Fast Output
  #if DEBUG
  for (int i = 0; i < 12; ++i)
  {
    Serial.print(i + 1);
    Serial.print("\t");
    Serial.print(": ");
    for (int j = 0; j < 16; ++j)
    {
      Serial.print((valArray[i][j] - REFERENCE) > 48 || (valArray[i][j] - REFERENCE) < -48 ? ((valArray[i][j]) - REFERENCE) : 0);
      // Serial.print((valArray[i][j] - REFERENCE) < -THRESHOLD || (valArray[i][j] - 2000) > THRESHOLD);
      Serial.print(j == 15 ? "" : "\t");
    }
    Serial.println();
  }
  Serial.println("---");
  delay(50);
  
  fillBuf(valArray);
  sendBinaryState(valArray);
  #endif
}
