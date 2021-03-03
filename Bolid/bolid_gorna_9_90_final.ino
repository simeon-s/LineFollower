/*
  Author :  Simeon Simeonov
  Name :  Bolid  Arduino project
  Ver:  Dancho micro optimizing
*/

/* Празници на роботиката и мехатрониката Русе 2017
 * Следене на линия резултат около 11.60
 * Параметрите са ориентировъчни
 * 
 * 
 * 
 * 
 */


//---------- NEC REMOTES -------------
//#define IR_NEC_REMOTE1  //сивото
#define IR_NEC_REMOTE2  //черното


#include <Arduino.h>
#include <PWM.h>
#include "pitches.h"
#include "nec_ir_commands.h"

//--------------------- PIN definitions -----------
#define RIGHT_EDGE  A2      // A0
#define RIGHT_POS   A3      // A1
#define CENTERR_POS A4      // A2
#define CENTERL_POS A5      // A2
#define LEFT_POS    A6      // A3
#define LEFT_EDGE   A7      // A4

#define LEFT_PWM    9
#define RIGHT_PWM   10
#define LEFT_DIR    5
#define RIGHT_DIR   6


//--------------- LEDS -----------
#define GREEN_LED  0      // LED1
#define RED_LED    1      // LED2

//-------------- Optrons ---------
#define OPT_ENABLE_ONE    2
#define OPT_ENABLE_TWO    3
#define OPT_ENABLE_THREE  4

//-------------- Beep ----------
#define BEEP        13

//--------------  Buttons -----------
#define BUTT1     7
#define BUTT2     8

//--------------  Infrared -----------
#define IR_PIN        8
//-------------------------- Sensors and position -------------------------
#define SENSORS_NR  6
const unsigned int sensors[SENSORS_NR] = { A7, A6, A5, A4, A3, A2 };  //left-right

//---------------- Състояние ---------
#define FORWARD    1
#define LEFT       2
#define RIGHT      3
#define BACK       4


///////////////////////////////////////////////////////////////////////////
//----------------------------------- PID ---------------------------------
///////////////////////////////////////////////////////////////////////////
#define PWM_FREQ      50000
//#define BREAKING

#define LINE_TRESHOLD   45//40    // Ниво под което се смята че няма линия под сензора
#define SENSOR_TRESHOLD 8 //5    // Филтриране на шумове от сензорите

//------------------  1000 rpm -------------
#define KP             0.5 // 10.01  
#define KD             5
#define KQ             10   
#define MAX_SPEED      145 //
#define ACQUIRE_SPEED  25


/*

//------------------  1000 rpm -------------
#define KP             0.5 // 10.01  
#define KD             5
#define KQ             10   
#define MAX_SPEED      140 //
#define ACQUIRE_SPEED  25
//10.27

*/
/*
#define KP             0.5 // 10.01  
#define KD             5
#define KQ             10   
#define MAX_SPEED      170 
#define ACQUIRE_SPEED  25
*/

//

//--------
#define SLOW_SPEED     MAX_SPEED/4// - MAX_SPEED/4  //3.4 Стабилизирне след завръщане на линията MAX_SPEED - MAX_SPEED/4
#define BREAK_LEVEL    MAX_SPEED * 0.6             // 0.6
//#define BREAK_LEVEL     25
#define BREAK_SPEED     -36

//  Скорости на моторите при изпуснат завой - търсене на линията в ляво или дясно
// Two speed for turn - for making right turn radius

#define TURN_SPEED_HIGH     MAX_SPEED + MAX_SPEED/7 //6 //7
#define TURN_SPEED_LOW      -20 //16 24 32

// Лява и дясна позиция, след които се изпуска линията
#define TURN_ERROR_LEFT    100    //  100 minimum
#define TURN_ERROR_RIGHT   600    //  600 maximum

/////////////////////////////////////////////////////////////////////////
//------- Global Variables -----
signed long position;
int dir;
unsigned int sensors_sum;
signed long quadr;
signed int error[5]; //faster readings => longer error chain

signed int last_position;
signed int left_pwm;
signed int right_pwm;
signed int correction;

//=================  CALIBRATE ==========================
unsigned int sensor_values[SENSORS_NR];
unsigned int sensor_calmin[SENSORS_NR];
unsigned int sensor_calmax[SENSORS_NR];
unsigned int sensor_denom[SENSORS_NR];

//--------- Common ------------
char tmp_str[64];

// prascaler bytes
byte PS_16 = (1 << ADPS2);
//byte PS_32 = (1 << ADPS2) | (1 << ADPS0);
//byte PS_64 = (1 << ADPS2) | (1 << ADPS1);
byte PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//-------------- IR remote control --------------------
#define  IR_CMD_START   KEY_OK
#define  IR_CMD_STOP    0x15

typedef struct {
  unsigned char address_low;
  unsigned char address_high;
  unsigned char command;
  unsigned char command_n;
} IRDATA;

volatile static uint8_t  ir_status;
volatile static uint8_t bits;
volatile IRDATA ir_data;
volatile unsigned long *ir_command;
volatile static uint8_t ir_data_get;    //  Флаг, който показва че име вече приета команда. Трябва след прочитане да се нулира

//---------------
unsigned long period;

//////////////////////////////////////////////////////////////////////////////
//                          SETUP
//////////////////////////////////////////////////////////////////////////////

//     Първоначална инициализация на всички входове, изходи и променливи
void setup() {
  pinMode(RIGHT_EDGE, INPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  // Управление на оптроните на 3 секции х 2
  pinMode( OPT_ENABLE_ONE , OUTPUT);
  pinMode( OPT_ENABLE_TWO , OUTPUT);
  pinMode( OPT_ENABLE_THREE , OUTPUT);

  // --- Изходи  ---
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BEEP, OUTPUT);

  // --- Входове ---
  pinMode(BUTT1, INPUT_PULLUP);
  pinMode(BUTT2, INPUT_PULLUP);
  
  //---- PWM инициализация за управление на моторите
  InitTimersSafe();
  if ( SetPinFrequencySafe(LEFT_PWM, PWM_FREQ))
  {
    pinMode(LEFT_PWM, OUTPUT);
  }
  if (SetPinFrequencySafe(RIGHT_PWM, PWM_FREQ))
  {
    pinMode(RIGHT_PWM, OUTPUT);
  }
  
  // set sample rate to XXXX X001 => 16 micros readings
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_16;
  
  // initialize serial communication at 9600 bits per second:
//  Serial.begin(9600);

  buzz(2400, 400);

  delay(2000);
  digitalWrite(BEEP, LOW);
  
  // Изчакване натискането на старт от дистанционно 
  pinMode(IR_PIN, INPUT_PULLUP);
  pciSetup(IR_PIN);
  
  // Изчакване натискането на бутон
  while (digitalRead (BUTT1) == HIGH) {
    if (ir_data_get) {
      ir_data_get = 0;
      Serial.print("IR command 0x" );
      Serial.println(ir_data.command, HEX);
      if (ir_data.command == IR_CMD_START)
        break;
    }
  }
  
  callibrate();
  
  dir = FORWARD;
  error[4] = 0;
  error[3] = 0;
  error[2] = 0;
  error[1] = 0;
  
  // ---  Изпълняване на мелодията ---
  play_melody();
  
  // Изчакване натискането на бутон или команда от старт модул
  // Спиране ако има команда от Старт модул
  ir_data_get = 0;
  ir_data.command = 0;
  while ((ir_data.command != IR_CMD_START) && (digitalRead (BUTT1) == HIGH) ) {
    if (ir_data_get) {
      ir_data_get = 0;
    }
  }
}


//============================================================================
//
// the loop routine runs over and over again forever:
//
//============================================================================
void loop() {
  //---- Прочитане на текущата позиция и пресмятане на грешката ----
  position = read_position();
  error[0] = position - 350;

  switch (dir) {
    case FORWARD:    //Движение направо
    default:
   
      if (position < TURN_ERROR_LEFT) {  //Ако отклонението е много голямо преминаване в състояние "Завой наляво"
#ifdef BREAKING        
        //--- Break ---
        right_motor_speed (BREAK_SPEED);
        left_motor_speed (BREAK_SPEED);
        delay(15);
#endif
        // --- Turn ---
        right_motor_speed (TURN_SPEED_HIGH);
        left_motor_speed (TURN_SPEED_LOW);
        //Serial.println(">>>>>>>>>>>>>>>>>>>>>>>");
        dir = LEFT;
        break;
      }

      if (position > TURN_ERROR_RIGHT) {  //Ако отклонението е много голямо преминаване в състояние "Завой надясно"
#ifdef BREAKING        
        //--- Break ---
        right_motor_speed (BREAK_SPEED);
        left_motor_speed (BREAK_SPEED);
        delay(15);
#endif
        // --- Turn ---
        right_motor_speed (TURN_SPEED_LOW);
        left_motor_speed (TURN_SPEED_HIGH);
        //Serial.println("<<<<<<<<<<<<<<<<<<<<<");
        dir = RIGHT;
        break;
      }


      //-------------PD закон за управление---------------
      quadr =  error[0] * error[0] * KQ;
      quadr = quadr / 10000;
      if (quadr < 0)
         quadr = ~--quadr; // magic
         
      correction = KP * error[0] + KD * (error[0] - error[3]) + quadr;
      error[3] = error[2];
      error[2] = error[1];
      error[1] = error[0];

      left_pwm = MAX_SPEED + correction;
      right_pwm = MAX_SPEED - correction;


    // ако MAX_SPEED > 140 трябва да се спира единия мотор.
      if (left_pwm < BREAK_LEVEL)
        left_pwm = BREAK_SPEED;
      if (right_pwm < BREAK_LEVEL )
        right_pwm = BREAK_SPEED;

        
      //---- Задаване на пресметнатата скорост на моторите ----
      left_motor_speed(left_pwm);
      right_motor_speed(right_pwm);
      break;

    case LEFT:     // Завой наляво при голямо отклонение - търсене на линия
      if (position > TURN_ERROR_LEFT + 50) {    //Maximum is 100%
        left_motor_speed (MAX_SPEED);       // Завръщане обратно на линията
        right_motor_speed (SLOW_SPEED);     // Спирачен момент в обратна посока (за предотвратяване на заклащането)

        dir = FORWARD;
        error[3] = 0;
        error[2] = 0;
        error[1] = 0;
        last_position = 0;
        delay (10);
      }
      break;


    case RIGHT:     // Завой надясно при голямо отклонение - търсене на линия
      if (position < TURN_ERROR_RIGHT - 50) {    //Maximum is 100%
        left_motor_speed (SLOW_SPEED);       // Завръщане обратно на линията
        right_motor_speed (MAX_SPEED);     // Спирачен момент в обратна посока (за предотвратяване на заклащането)

        dir = FORWARD;
        error[3] = 0;
        error[2] = 0;
        error[1] = 0;
        last_position = 0;
        delay (10);
      }
      break;

  }
  
  //------------ Спиране ако има команда от Старт модул -----------
  //трябва да се редактира
  if (ir_data_get) {
    ir_data_get = 0;
    left_motor_speed (0);       // STOP
    right_motor_speed (0); 
   
    if (ir_data.command == IR_CMD_STOP) {
      while ((ir_data.command != IR_CMD_START)) {
        if (ir_data_get)
          ir_data_get = 0;
      }
    }
  }
    
}

//=======================================================================================

/* -----------------------------------------------------------------------------------
    Two way PWM speed control. If speed is >0 then motor runs forward.
    Else if speed is < 0 then motor runs backward
  ------------------------------------------------------------------------------------ */
void left_motor_speed(signed int motor_speed) {
  if (motor_speed == 0)
    motor_speed = 1;

  if (motor_speed > 0) {
//    digitalWrite(LEFT_DIR, HIGH);
    PORTD  |= B00100000; // set LEFT_DIR on HIGH
  }
  else {
//    digitalWrite(LEFT_DIR, LOW);
    PORTD  &= B11011111; // set LEFT_DIR on LOW
  }
  motor_speed = abs(motor_speed);
  if (motor_speed > 255)
    motor_speed = 255;

  pwmWrite(LEFT_PWM, motor_speed);
}

//---------------------------------------
void right_motor_speed(signed int motor_speed) {
  if (motor_speed == 0)
    motor_speed = 1;

  if (motor_speed > 0) {
//    digitalWrite(RIGHT_DIR, HIGH);
    PORTD |= B01000000; // set RIGHT_DIR on HIGH
  }
  else {
//    digitalWrite(RIGHT_DIR, LOW);
    PORTD &= B10111111; // set RIGHT_DIR on LOW
  }
  motor_speed = abs(motor_speed);
  if (motor_speed > 255)
    motor_speed = 255;

  pwmWrite(RIGHT_PWM, motor_speed);
}


//----------------------Sofware-tone-without-using-arduino-buildin-timers-------------------------
void play_melody() {
  byte thisNote;
  int noteDuration;
  int pauseBetweenNotes;
  
    // melody and tempo declarated in pitches.h
    for (thisNote = 0; thisNote < 20; thisNote++) {
      noteDuration = 1000 / tempo[thisNote];    
      buzz(melody[thisNote], noteDuration);
      pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      buzz(0, noteDuration);
    }
}

void buzz (unsigned long frequency, unsigned long length) {
  unsigned long delayValue = 1000000 / frequency / 2;
  unsigned long numCycles = frequency * length / 1000;
  
  for (long i = 0; i < numCycles; i++) {
    PORTB |= B100000;
    delayMicroseconds(delayValue);
    PORTB &= B011111;
    delayMicroseconds(delayValue);
  }
  PORTB &= B011111;
}

//===============================  CALIBRATE ======================================
//-------------------
void callibrate(void) {
  int i, sens;
  int center_pos;
  unsigned int tmp_value;

  //--- Preset the arrays ---
  for (i = 0; i < SENSORS_NR; i++)
    (sensor_calmin)[i] = 4095;    // Maximum ADC value

  for (i = 0; i < SENSORS_NR; i++) {
    (sensor_calmax)[i] = 0;       // Minimum ADC value
    tmp_value = analogRead(sensors[i]);
  }

  //--- Turn right ---
  left_motor_speed(ACQUIRE_SPEED);      // forward
  right_motor_speed(-ACQUIRE_SPEED);    // backward

  for (i = 0; i < 1400; i++) {
    //----- Find minimum and maximum values for all sensors -----
    for (sens = 0; sens < SENSORS_NR; sens++) {
      if (( sens == 0) || ( sens == 3)) {
//        digitalWrite( OPT_ENABLE_ONE , HIGH);
//        digitalWrite( OPT_ENABLE_THREE , LOW);
        PORTD  = (PORTD & B11101111) | B00000100; // set OPT_ENABLE_THREE on LOW and OPT_ENABLE_ONE on HIGH
      } else if ((sens == 1) || (sens == 4)) {
//        digitalWrite( OPT_ENABLE_ONE , LOW);
//        digitalWrite( OPT_ENABLE_TWO , HIGH);
        PORTD  = (PORTD & B11111011) | B00001000; // set OPT_ENABLE_ONE on LOW and OPT_ENABLE_TWO on HIGH
      } else {
//        digitalWrite( OPT_ENABLE_TWO , LOW);
//        digitalWrite( OPT_ENABLE_THREE , HIGH);
        PORTD  = (PORTD & B11110111) | B00010000; // set OPT_ENABLE_TWO on LOW and OPT_ENABLE_THREE on HIGH
      }
      delayMicroseconds(150);

      tmp_value = analogRead(sensors[sens]) >> 1;
      if (tmp_value < sensor_calmin[sens])
        sensor_calmin[sens] = tmp_value;
      if (tmp_value > sensor_calmax[sens])
        sensor_calmax[sens] = tmp_value;
    }

    if (i == 550) {   // --- turn left  ---
      left_motor_speed(-ACQUIRE_SPEED);
      right_motor_speed(ACQUIRE_SPEED);
    }
//    delay(1);
  }

//  digitalWrite( OPT_ENABLE_ONE , HIGH);
//  digitalWrite( OPT_ENABLE_THREE , LOW);
  PORTD  = (PORTD & B11101111) | B00000100; // set OPT_ENABLE_TWO on LOW and OPT_ENABLE_THREE on HIGH
  //-------   Calculate calibration  denom --------
  for (sens = 0; sens < SENSORS_NR; sens++) {
    sensor_denom[sens] = (sensor_calmax[sens] - sensor_calmin[sens]) / 10;

//    sprintf(tmp_str, " min %d - max %d", sensor_calmin[sens], sensor_calmax[sens]);
//    Serial.println(tmp_str);
  }

  //---------- Go back to the line ----------
  left_motor_speed(ACQUIRE_SPEED);
  right_motor_speed(-ACQUIRE_SPEED);
  delay(1);
  
  do {
    tmp_value = analogRead(sensors[3]) >> 1;
    center_pos = ((tmp_value - sensor_calmin[3]) * 10) / sensor_denom[3];// Center sonsor position in the array = 2
    delay(10);
  } while (center_pos < 80);

  // --- Strom the motors ---
  left_motor_speed(0);
  right_motor_speed(1);
  
  PORTD  &= B11100011; // turn off light
//  digitalWrite( OPT_ENABLE_ONE , LOW);
//  digitalWrite( OPT_ENABLE_TWO , LOW);
//  digitalWrite( OPT_ENABLE_THREE , LOW);
}

//============================= Read sensors  and scale =====================
unsigned int read_position(void) {
  unsigned char on_line;
  static unsigned int last_pos;
  unsigned char sens;
  unsigned int tmp_value;
  signed long pos;

  pos = 0;
  sensors_sum = 0;
  on_line = 0;

  //-------------- Read sensors ------------
  for (sens = 0; sens < 3; sens++) {
    if ( sens == 0) {
//      digitalWrite( OPT_ENABLE_ONE , HIGH);
      PORTD  |= B00000100;                      // set OPT_ENABLE_ONE on HIGH
    } else if (sens == 1) {
//      digitalWrite( OPT_ENABLE_ONE , LOW);
//      digitalWrite( OPT_ENABLE_TWO , HIGH);
      PORTD  = (PORTD & B11111011) | B00001000; // set OPT_ENABLE_ONE on LOW and OPT_ENABLE_TWO on HIGH
    } else {
//      digitalWrite( OPT_ENABLE_TWO , LOW);
//      digitalWrite( OPT_ENABLE_THREE , HIGH);
      PORTD  = (PORTD & B11110111) | B00010000; // set OPT_ENABLE_TWO on LOW and OPT_ENABLE_THREE on HIGH
    }
    delayMicroseconds(150);    // Wait for lighting
    tmp_value = analogRead(sensors[sens]) >> 1; // bit shift by 1 to left == (/2)

    //--------- Validate ----------
    if (tmp_value < sensor_calmin[sens])
      tmp_value = sensor_calmin[sens];
    if (tmp_value > sensor_calmax[sens])
      tmp_value = sensor_calmax[sens];

    //-------- Calibrate ----------
    sensor_values[sens] = ((tmp_value - sensor_calmin[sens]) * 10)
                          / sensor_denom[sens];

    //----------- Noise filtering ----------
    if (sensor_values[sens]  < SENSOR_TRESHOLD)
      sensor_values[sens] = 0;


    // The estimate position is made using a weighted average of the sensor indices
    // multiplied by 100,  The formula is:
    //
    //    100*value0 + 200*value1 + 300*value2 + ...
    //   --------------------------------------------
    //         value0  +  value1  +  value2 + ...

    pos += sensor_values[sens] * ((sens + 1) * 100);
    sensors_sum += sensor_values[sens];

    //--- line presens check and count ---
    if (sensor_values[sens] > LINE_TRESHOLD)
      on_line += 1;

    //================================================
    tmp_value = analogRead(sensors[sens + 3]) >> 1; // bit shift by 1 to left == (/2)

    //--------- Validate ----------
    if (tmp_value < sensor_calmin[sens + 3])
      tmp_value = sensor_calmin[sens + 3];
    if (tmp_value > sensor_calmax[sens + 3])
      tmp_value = sensor_calmax[sens + 3];

    //-------- Calibrate ----------
    sensor_values[sens + 3] = ((tmp_value - sensor_calmin[sens + 3]) * 10)
                              / sensor_denom[sens + 3];

    //----------- Noise filtering ----------
    if (sensor_values[sens + 3]  < SENSOR_TRESHOLD)
      sensor_values[sens + 3] = 0;

    // The estimate position is made using a weighted average of the sensor indices
    // multiplied by 100,  The formula is:
    //
    //    100*value0 + 200*value1 + 300*value2 + ...
    //   --------------------------------------------
    //         value0  +  value1  +  value2 + ...

    pos += sensor_values[sens + 3] * ((sens + 1 + 3) * 100);
    sensors_sum += sensor_values[sens + 3];

    //--- line presens check ---
    if (sensor_values[sens + 3] > LINE_TRESHOLD)
      on_line += 1;
  }
//  digitalWrite( OPT_ENABLE_THREE , LOW);
  PORTD &= B11101111; // set OPT_ENABLE_THREE on LOW

  if (!on_line) {
    // If it last read to the left of center, return 0.
    //if (last_pos < (SENSORS_NR - 1) * 100 / 2) {
    if (last_pos < 250) {
      last_pos = 90;
      // If it last read to the right of center, return the max.
    }
    else if (last_pos > 450) {
      last_pos = (SENSORS_NR * 100) + 10;
    } else {
      last_pos = 350;   // center pos
    }
  }
  else {
    if (on_line > 2) {
      tmp_value = sensor_values[0] + sensor_values[1] + sensor_values[2];
      if (  tmp_value > 90 )
        last_pos = 100;
      tmp_value = sensor_values[SENSORS_NR-3] + sensor_values[SENSORS_NR-2] + sensor_values[SENSORS_NR-1];
      if  ( tmp_value > 90 )
        last_pos = (SENSORS_NR * 100);

    } else
      last_pos = pos / sensors_sum;
  }

  return last_pos;
}


//===========================================================================
// http://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol#
//===========================================================================
/*
   Interrupt stuff
*/

#define IR_STARTBIT1  0
#define IR_STARTBIT2  1
#define IR_ADDRL    2
#define IR_ADDRH    3
#define IR_CMD      4
#define IR_CMDN     5




//-----------------------------------------
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// //Use one Routine to handle each group

ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{
  static unsigned char edge = 0;
  unsigned long ir_period;

  if (digitalRead(IR_PIN) == LOW) {
    ir_period = (micros() - period) / 100;
    period = micros();
    edge = 0;
  } else {
    edge = 1;
  }

    switch (ir_status) {
    case IR_STARTBIT1:
      // Workaround to interrupt on any edge
      if (!edge) {
        edge = 1;
      } else {
        ir_period = (micros() - period) / 100;
        period = micros();
        // 9ms leading pulse burst (16 times the pulse burst length used for a logical data bit)
        if ((ir_period >= 85) && (ir_period < 95) && !ir_data_get)
          ir_status = IR_STARTBIT2;
        else {
//          Serial.print("Period 1 = ");
//          Serial.println(ir_period);
        }
        edge = 0;
      }
      break;

    case IR_STARTBIT2:
      if (!edge) {
        //  4.5ms space
        if ((ir_period > 40) && (ir_period < 50)) {
          ir_status = IR_CMD;
          edge = 0;
          ir_command = (unsigned long*) &ir_data;
          *ir_command = 0;
          //ir_data_get = 0;
          bits = 0;
//          Serial.println("IR_ CMD");
        }
        else {
          ir_status = IR_STARTBIT1;
//          Serial.print("Period 2 = ");
//          Serial.println(ir_period);
          return;
        }
      }
      break;

    case IR_CMD:
      if (!edge) {
        if ((bits < 32) && (ir_period < 30)) {
          ir_command = (unsigned long*) &ir_data;
          if ((ir_period > 8) && (ir_period < 15)) {
            *ir_command >>= 1;
            //Serial.write("0",1);
          }
          else if (ir_period > 16) {
            *ir_command >>= 1;
            *ir_command |= 0x80000000;
            //Serial.write("1",1);
            //Serial.print(ir_period);
          }
          bits++;
        }
        if (bits >= 32) {               // addrL + AddrH + cmd + cmdN = 32 bit
          ir_data_get = 1;
          ir_status = IR_STARTBIT1;
//          Serial.println("IR got data");
          return;
        }

        if (ir_period > 30) {
          ir_status = IR_STARTBIT1;
//          Serial.println("IR wrong data");
          return;
        }
      }
      break;
  }
}


ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  digitalWrite(13, digitalRead(A0));
}


ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
  //digitalWrite(GREEN_LED, digitalRead(IR_PIN));

  static unsigned char edge = 0;
  unsigned long ir_period;

  if (digitalRead(IR_PIN) == LOW) {
    ir_period = (micros() - period) / 100;
    period = micros();
    edge = 0;
  } else {
    edge = 1;
  }
}
//=============================================================================
