/*        Your Name & E-mail: Kunal Shrivastav & kshri003@ucr.edu

*          Discussion Section: 024

 *         Assignment: Lab #7  Exercise #3

 *         Exercise Description: Rotate servo motor on RIGHT and LEFT in joystick
 *        

 *         I acknowledge all content contained herein, excluding template or example code, is my own original work.

 *

 *         Demo Link:  https://youtu.be/jFlPRMtkWjo

 */
#include "timerISR.h"
#include "helper.h"
#include "periph.h"

#define NUM_TASKS 5 //TODO: Change to the number of tasks being used
#define TASK_JOYSTICK_PERIOD 1
#define TASK_SWITCH_PERIOD 100
#define TASK_BLINK_PERIOD 100
#define LED_TIMER 10000
#define TASK_STEP_PERIOD  2
#define TASK_SERVO_PERIOD 1

#define SWITCH1 3
#define SWITCH2 4
#define SWITCH_ON 1
#define SWITCH_OFF 0

int phase_counter = 0;
int step_motor_direction = 0 ;
int step_motor_speed = 1 ;

int servo_motor_direction = 0 ;

enum Step_Motor_States {STEP_IDLE, STEP_FWD, STEP_REVERSE } Step_Motor_State;
enum Servo_Motor_States {SERVO_IDLE, SERVO_FWD, SERVO_REVERSE } Servo_Motor_State;

//Task struct for concurrent synchSMs implmentations
typedef struct _task{
	signed 	 char state; 		//Task's current state
	unsigned long period; 		//Task period
	unsigned long elapsedTime; 	//Time elapsed since last task tick
	int (*TickFct)(int); 		//Task tick function
} task;

#define PORT_OFF 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3
#define PORT_ALL 8

/*0 = off, 1 = PORTB, 2 = PORTC, 3 = PORTD*/
int (*currPattern)[3]; //when want to blink, set this to pattern
bool blinkOn = false; //turn it to true when we press the switch, then assign currPattern to whatever I want.
int arrayOfPatterns0[4][3] = {{PORT_D, 4, LED_TIMER},{PORT_D,3,LED_TIMER}, {PORT_D, 2, LED_TIMER}, {PORT_ALL, 2, LED_TIMER}};
int arrayOfPatterns1[4][3] = {{PORT_B, 0, LED_TIMER},{PORT_D,7,LED_TIMER}, {PORT_D, 5, LED_TIMER}, {PORT_ALL, 5, LED_TIMER}};


//TODO: Define Periods for each task
// e.g. const unsined long TASK1_PERIOD = <PERIOD>
const unsigned long GCD_PERIOD = 1;//TODO:Set the GCD Period
//bool beepon = false;
task tasks[NUM_TASKS]; // declared task array with 5 tasks
//prescaler range (2,3,4,5)
int TCCR0A_save ;
void Beep_On(int prescaler) {
  OCR0A = 128; //sets duty cycle to 50% since TOP is always 256
  TCCR0B = (TCCR0B & 0xF8) | prescaler; //set prescaler to 8
}

unsigned int rmap_value(unsigned int aFirst, unsigned int aSecond, unsigned int bFirst, unsigned int bSecond, unsigned int inVal)
{
    return bFirst - (long((inVal - aFirst)) * long((bFirst - bSecond))) / (aSecond - aFirst);
}

void Beep_Off() {
  OCR0A = 255;
  //TCCR0A = 0 ;
  //TCCR0B = (TCCR0B & 0xF8) | 0x00;//set prescaler to 0
}

void TimerISR() {
	for ( unsigned int i = 0; i < NUM_TASKS; i++ ) {                   // Iterate through each task in the task array
		if ( tasks[i].elapsedTime == tasks[i].period ) {           // Check if the task is ready to tick
			tasks[i].state = tasks[i].TickFct(tasks[i].state); // Tick and set the next state for this task
			tasks[i].elapsedTime = 0;                          // Reset the elapsed time for the next tick
		}
		tasks[i].elapsedTime += GCD_PERIOD;                        // Increment the elapsed time by GCD_PERIOD
	}
}
int stages[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};//Stepper motor phases

//TODO: Create your tick functions for each task



enum joystick_states{JOYSTICK_INIT, IDLE, UP, DOWN, REMAIN_UP, REMAIN_DOWN, RIGHT, REMAIN_RIGHT, LEFT, REMAIN_LEFT, BUTTON_PRESS, BUTTON_RELEASE} joystick_state;
int Tick_JoyStick(int state) {
    static unsigned int beepCounter = 0; // Counter to manage the beep timing

    int adcval = 0;
    int adcval1 = 0 ;
    /*state transitions*/
    switch(state)
    {
        case JOYSTICK_INIT:
            state = IDLE;
            break;
        case IDLE:
            // A5 A4 A3 A2 A1 A0
            if((PINC >> 2 & 0x01) == 0) {
                state = BUTTON_PRESS;
           }
            else {
              adcval = ADC_read(0) ;
              adcval1 = ADC_read(1) ;
              if ( adcval > 750 ) { 
                  state = UP;
                  step_motor_direction = STEP_FWD ;
              }
              if ( adcval< 400 )  {
                  state = DOWN;
                  step_motor_direction = STEP_REVERSE ;
              }
              
              if ( adcval1 > 750 ) { 
                  state = RIGHT ;
                  servo_motor_direction = SERVO_FWD ;
              }
              if ( adcval1 < 400 )  {
                  state = LEFT;
                  servo_motor_direction = SERVO_REVERSE ;
              }
            }
            break;
        case UP:
        case DOWN:
        case REMAIN_UP:
        case REMAIN_DOWN:
            break;
        case RIGHT:
            if(ADC_read(1) <= 950) state = IDLE; 
            else state = REMAIN_RIGHT;
            break;
        case LEFT:
            if(ADC_read(1) >= 100) state = IDLE;
            else state = REMAIN_LEFT;
            break;
        case REMAIN_RIGHT:
            if(ADC_read(1) <= 950) state = IDLE;
            break;
        case REMAIN_LEFT:
            if(ADC_read(0) >= 100) state = IDLE;
            break;
        case BUTTON_PRESS:
            if((PINC >> 2 & 0x01) == 1) 
                state = BUTTON_RELEASE; 
            break ;
        case BUTTON_RELEASE:
            state = IDLE; 
            break;
        default: break;    
    }
    /*state actions*/
    switch(state)
    {
        case JOYSTICK_INIT:
            break;
        case IDLE:
            beepCounter = 0; // Reset the beep counter when idle
            //Beep_Off(); // Ensure the buzzer is off when idle
            break;
        case REMAIN_UP:
            break;
        case REMAIN_DOWN:
            break;
        case BUTTON_PRESS:
            Beep_On(4);
            break ;
        case DOWN:
            // Handle the buzzer beeping logic
           if (beepCounter % 2000 == 0) { // every 2 seconds (assuming GCD_PERIOD is 1 ms)
                Beep_On(3); // Turn on the beep
            } else if (beepCounter % 2000 == 1000) { // for 1 second (assuming GCD_PERIOD is 1 ms)
                Beep_Off(); // Turn off the beep
            }
            beepCounter++;
            adcval = ADC_read(0) ;
            if( adcval < 500) {
                state = DOWN;
                step_motor_speed = map_value(0, 500, 0, 10, adcval);
            } else {
                state = IDLE ;
                step_motor_direction = STEP_IDLE ;
            }
            break;
        case UP:
            adcval = ADC_read(0) ;
            if( adcval > 600 ) {
                state = UP;
                step_motor_speed = rmap_value(600, 1023, 10, 0, adcval);
            } else {
                state = IDLE ;
                step_motor_direction = STEP_IDLE ;
            }
            break;
        case RIGHT:
          adcval1 = ADC_read(1) ;
          if( adcval1 > 600 ) {
                state = RIGHT ;
            } else {
                state = IDLE ;
                servo_motor_direction = SERVO_IDLE ;
            }
          break ;
        case LEFT:
          adcval1 = ADC_read(1) ;
          if( adcval1 < 500 ) {
                state = LEFT ;
            } else {
                state = IDLE ;
                servo_motor_direction = SERVO_IDLE ;
            }
          break ;
        case REMAIN_RIGHT:
        case REMAIN_LEFT:
            break;
        case BUTTON_RELEASE:
            //step_motor_direction = STEP_IDLE ;
            Beep_Off();
            break;  
        default: break;    
    }
    return state;
}

enum switch_states{SWITCH_INIT, SWITCH1_PRESS, SWITCH1_RELEASE, SWITCH2_PRESS, SWITCH2_RELEASE} switch_state;
int Tick_Switch(int state) {
  //blinkOn = false; 
  switch (state)
  {
    case SWITCH_INIT:
      if((PINC >> SWITCH1 & 0x01) == SWITCH_ON) {
        //Beep_On(4);
        state = SWITCH1_PRESS;
        blinkOn = true;
        currPattern = arrayOfPatterns0;
      }
      if((PINC >> SWITCH2 & 0x01) == SWITCH_ON) { 
        //Beep_On(2);
        state = SWITCH2_PRESS;
        blinkOn = true;
        currPattern = arrayOfPatterns1;

      }
      break;
    case SWITCH1_PRESS:
    case SWITCH1_RELEASE:
    case SWITCH2_PRESS:
    case SWITCH2_RELEASE:
      break;
    default: break;
  }
  /*state actions*/
  switch(state)
  {
    case SWITCH1_PRESS:
      if((PINC >> SWITCH1 & 0x01) == SWITCH_OFF) {
        
          state = SWITCH_INIT;
          blinkOn = false;
          //currPattern = arrayOfPatterns0;
      } 
      break;
    case SWITCH2_PRESS:
      if((PINC >> SWITCH2 & 0x01) == SWITCH_OFF) {
        state = SWITCH_INIT;
        blinkOn = false;
        //currPattern = arrayOfPatterns1;
      } 
      break;
    default:
      break;
  }
  return state;
}

/*function to blink LED based on pattern*/
int blink_index = 0;
enum blink_states {BLINK_INIT, BLINK_OFF, BLINK_ON} blink_state;
int Tick_BlinkLED(int state)
{
  int i ;
  if (!blinkOn ) {
    if ( state != -1) {
      for (i=0; i< 6; i++) {
        switch ( currPattern[i][0]) {
          case PORT_B:
            PORTB = SetBit(PORTB, currPattern[i][1], 0);
            break ;
          case PORT_C:
            PORTC = SetBit(PORTC, currPattern[i][1], 0);
            break ;
          case PORT_D:
            PORTD = SetBit(PORTD, currPattern[i][1], 0);
            break ;
          case PORT_OFF:
          default:
            break ;
        }
      }
      state = -1 ;
      blink_index = 0;
    } 
    return state;
  }
    // initial state
  if ( state < 0 ) 
  {
    state = currPattern[blink_index][2]; //time
    switch ( currPattern[blink_index][0])
    {
      case PORT_B:
        PORTB = SetBit(PORTB, currPattern[blink_index][1], 1);
        break ;
      case PORT_C:
        PORTC = SetBit(PORTC, currPattern[blink_index][1], 1);
        break ;
      case PORT_D:
        PORTD = SetBit(PORTD, currPattern[blink_index][1], 1);
        break ;
      case PORT_ALL:
        PORTD = SetBit(PORTD, currPattern[0][1], 0);
        PORTD = SetBit(PORTD, currPattern[1][1], 0);
        PORTD = SetBit(PORTD, currPattern[2][1], 0);
        PORTB = SetBit(PORTB, 0, 0);
        blink_index++;
        if(blink_index > 3)
        {
          blink_index = 0 ;
        }
        state = 5000 ;
        break ;
      default:
            break ;
    }
    return state ;
  }

  // if state is greater than zero that means there is a ongoing counter
  // decrement the counter

  if (state > 0) 
    state-- ;
  
  // zero turn off state
  if( !state ) {
    switch ( currPattern[blink_index][0]) {
      case PORT_B:
        //PORTB = SetBit(PORTB, currPattern[blink_index][1], 0);
        break ;
      case PORT_C:
        //PORTC = SetBit(PORTC, currPattern[blink_index][1], 0);
        break ;
      case PORT_D:
        //PORTD = SetBit(PORTD, currPattern[blink_index][1], 0);
        break ;
      case PORT_OFF:
        //PORTD = SetBit(PORTD, currPattern[blink_index][1], 0);
      default:
        break ;
    }
    blink_index++;
    if(blink_index > 3)
    {
        blink_index = 0 ;
    }
    state = -1;  // back to initial state
  }
  return state;
}

int step_current_speed = 0 ;

int Tick_Step_Motor(int state) {
    int phases[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001}; //8 phases of the stepper motor step

    // static unsigned int beepCounter = 0; // Counter to manage the beep timing

    step_current_speed--;
    if (step_current_speed > 0)
        return state;

    if (step_current_speed < 0)
        step_current_speed = 0;

    switch (state) {
        case STEP_IDLE:
            step_current_speed = step_motor_speed;
            if (step_motor_direction == STEP_FWD)   // forward
                state = STEP_FWD;
            else if (step_motor_direction == STEP_REVERSE)   // reverse
                state = STEP_REVERSE;
            break;
        case STEP_FWD:
        case STEP_REVERSE:
        default:
            break;
    }
    // actions
    switch (state) {
        case STEP_REVERSE:
            PORTB = (PORTB & 0x03) | phases[phase_counter] << 2;
            phase_counter++;//increment to next phase
            if (phase_counter > 7) { //if all phases are completed, restart
                phase_counter = 0;
            }
           state = STEP_IDLE;
            break;
        case STEP_FWD:
            PORTB = (PORTB & 0x03) | phases[phase_counter] << 2;
            phase_counter--;
            if (phase_counter < 0) {
                phase_counter = 7;
            }
            state = STEP_IDLE;
            break;
        case STEP_IDLE:
            Beep_Off();
        default:
            break;
    }
    return state;
}


int Tick_Servo_Motor(int state) {
  switch (state ) {
    case SERVO_IDLE :
      if (servo_motor_direction == SERVO_FWD)   // forward
          state = SERVO_FWD;
      else if (servo_motor_direction == SERVO_REVERSE)   // reverse
          state = SERVO_REVERSE;
      break ;
    case SERVO_FWD:
    case SERVO_REVERSE:
    default:
      break ;
  }

  //actions
  switch (state ) {
    case SERVO_IDLE :
      break ;
    case SERVO_FWD:
      OCR1A = (4999+999) - map_value(0,1023,999,4999, ADC_read(1));
      state = SERVO_IDLE;
      break ;
    case SERVO_REVERSE:
      OCR1A = (4999+999) - map_value(0,1023,999,4999, ADC_read(1));
      state = SERVO_IDLE;
      break ;
    default:
      break ;
  }
  return state ;
}

int main(void) {
  //TODO: initialize all your inputs and ouputs

  ADC_init();   // initializes ADC

  DDRC = 0x00 ; PORTC = 0xFF;
  DDRD = 0xFF; PORTD = 0x00 ;
  DDRB = 0xFF; PORTB = 0x00 ;

  // Initialize the buzzer timer/pwm(timer0)

  //OCR0A = 128; //sets duty cycle to 50% since TOP is always 256
  TCCR0A |= (1 << COM0A1);// use Channel A
  TCCR0A |= (1 << WGM01) | (1 << WGM00);// set fast PWM Mode
  
  //TODO: Initialize the servo timer/pwm(timer1)
  TCCR1A |= (1 << WGM11) | (1 << COM1A1); //COM1A1 sets it to channel A
  TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); //CS11 sets the prescaler to be 8
  //WGM11, WGM12, WGM13 set timer to fast pwm mode
  ICR1 = 39999; //20ms pwm period

  tasks[0].period = TASK_JOYSTICK_PERIOD;
  tasks[0].state = JOYSTICK_INIT;
  tasks[0].elapsedTime = 0;
  tasks[0].TickFct = &Tick_JoyStick;

  tasks[1].period = TASK_SWITCH_PERIOD;
  tasks[1].state = SWITCH_INIT;
  tasks[1].elapsedTime = 0;
  tasks[1].TickFct = &Tick_Switch;

  tasks[2].period = TASK_BLINK_PERIOD;
  tasks[2].state = -1;
  tasks[2].elapsedTime = 0;
  tasks[2].TickFct = &Tick_BlinkLED;

  tasks[3].period = TASK_STEP_PERIOD;
  tasks[3].state = STEP_IDLE ;
  tasks[3].elapsedTime = 0;
  tasks[3].TickFct = &Tick_Step_Motor;

  tasks[4].period = TASK_SERVO_PERIOD;
  tasks[4].state = SERVO_IDLE ;
  tasks[4].elapsedTime = 0;
  tasks[4].TickFct = &Tick_Servo_Motor;

  TimerSet(GCD_PERIOD);
  TimerOn();

  while (1) {}
  return 0;
}