/******************************************************************************
 Title:    "servo controller" no feedback, stepper motor output, proportionate control, RC pwm in
 
 
 Author:   rue_mohr
 Date:     june 2023
 Software: AVR-GCC 3.3 
 Hardware: attiny13 @ 9.6Mhz
 
    

PB0 stepper driver STEP
PB1 stepper driver DIR
PB2 stepper driver /EN
PB3 
PB4 Hobby PWM IN

PB5 (RESET)   


                            +-----U-----+    
               RESET    PB5 | o         | VCC
               ADC3     PB3 |           | PB2 ADC1 
               ADC2     PB4 |   Tiny13  | PB1 OC0B
                        GND |           | PB0 OC0A
                            +-----------+    


Motor step driver set to 8 microstep.
Max step rate is       20kHz
Min step rate is about 5kHz.

min input pulse accepted is 0.5ms
max input pulse accepted is 3.0ms
The code does not care about the time between control pulses, tho a minimum does apply.

timer 0 is used for dividing the system clock to generate the step timings
ADC is used as a acceleration update timer, and control pulse timeout timer.

main loop measures the servo pulses.


    
*******************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "avrcommon.h"
#include <stdint.h>

#define STEPBIT  0
#define DIRBIT   1
#define ENBIT    2

#define PWM0PORT PORTB
// valid tuned to 0.5ms - 3ms (why is my remote going up to 3ms!? *WHATEVER*)
// units are uberBogoMIPS
#define MINVALIDCOUNT  450
#define MAXVALIDCOUNT  2500

// timeout at two missing control pulses.
// units are ADC conversions
#define TIMEOUT  1500 



volatile uint8_t  waitCount;
volatile uint16_t timeout;   
int16_t ctrl;
volatile int16_t fb;

int8_t  dir;
uint16_t rate;

void ADCInit() ;
void TimerInit() ;
void setSpeed( int v ) ;


int main (void)  {

  int      t;     // target velocity
  int      v;     // current velocity
  uint16_t buff;  // control measure buffer
  uint8_t  oi, i;
  
  // Set clock prescaler: 0 gives full 9.6 MHz from internal oscillator.
  CLKPR = (1 << CLKPCE);
  CLKPR = 0;  
  
  dir   = 0;
  rate  = 255;
         
  DDRB = (OUTPUT << DDB0 | OUTPUT << DDB1 | OUTPUT << DDB2 | INPUT << DDB3 | INPUT << DDB4 | INPUT << DDB5 ); 
  
  SetBit   ( ENBIT, PORTB ); // disable stepper
  
  TimerInit();
  ADCInit();  
  sei();         // turn on interrupts    

  ctrl = 0;
  while (ctrl == 0) {  
    buff = 0;
    while(IsHigh(4, PINB) && (buff < MAXVALIDCOUNT+2)) buff++;  // time control pulse
    if (inBounds(buff, MINVALIDCOUNT, MAXVALIDCOUNT)) ctrl = buff;    // valid pulses from 0.5ms to 2ms
  }
  
  fb   = ctrl;
  v    = 0;            
  buff = 0;    
  
  oi = IsHigh(4, PINB);
  i = oi;
  
  ClearBit( ENBIT, PORTB ); // enable stepper   
          
  while(1) { // main loup ;]
    
    while(1) { // do velocity control until rising edge of control     
     
      if (waitCount == 0) {           // only do calcs if its time to
      
        t = (fb - ctrl)/2;             // proportionate calc, where the error is used to control motor velocity                      
        t = limit(t, -255, 255);       // limit target velocity

      //  v += limit((t - v), -1, 1); // step velocity, acceleration determined by waitCalcLoop    
        v += SIGN((t - v));

        setSpeed( v );         
        waitCount = 5; // this sets acceleration limit.
      }
        
      oi = i; 
      i  = IsHigh(4, PINB);     
      
      if (timeout == 0)    SetBit   ( ENBIT, PORTB ); // disable stepper if we timed out     
      
      if ((oi == 0) && (i == 1)) break; // yea I know, I'm having a bad boolean day ok?
    }     
    
    // control pulse is present, measure and update control
    
    while(IsHigh(4, PINB) && (buff < MAXVALIDCOUNT+2)) buff++;  // time control pulse, don't bother above max+2
    
    if (inBounds(buff, MINVALIDCOUNT, MAXVALIDCOUNT)) {
      
      //ctrl = buff;                                 // valid pulses from 0.5ms to 2ms
      
      ctrl = (ctrl + buff)/2;                        // a bit of filtering. (this helps a LOT!)
      
      timeout = TIMEOUT;                             // reset timeout with valid pulse
      ClearBit ( ENBIT, PORTB );                     // enable stepper
      
      // it might be a good thing to add code to require a number of valid pulses before the motor is enabled.
      
    }
    buff = 0;       
             
   }
}

    
//------------------------| FUNCTIONS |------------------------




/*
  OK

  Max freq 20kHz   ( measured motor data )
  Max start 9.5kHz ( measured motor data )

 v input range is -255 to 255
 
 output values are 
    dir (-1, 0, 1)
    rate (480 -> 10000) inverse for a clock divider to timer 1 (16 bit)
*/
void setSpeed( int v ) {

  dir = 1;
  if (v < 0) {
    dir = -1;
    v = -v;
  }  

  if (v == 0) {
    dir = 0;
    return;
  } 
  
  v *= 49; // not blowing the 16 bit math yet!
  v = 16335 - v;
  v /= 64;
  rate = v; 
 
}

/* OK */
// the ADC is just being used as a timer.
void ADCInit() {

  // ADC, 9.6Mhz / 32
  
  ADMUX  = ( (1 << MUX1) ); 
  
  ADCSRA = ( (1 << ADEN) | (1 << ADIF) | (1 << ADIE) | (5 << ADPS0)  | (1 << ADATE) | (1 << ADSC)  ) ; 

} 

/*
  OK
  
  Timer 0 as a rate generator
  
*/
void TimerInit() {
  
  OCR0A  = 255;      
  
  TCCR0B = (2<<CS00); // timer 1 using /8 clock
  TCCR0A = (1<<WGM01);
  SetBit( OCIE0A, TIMSK0 ); // enable timer
    
}


// ----------------------| ISR |------------------------------

/* ok */
// occurs at about 32.1 kHz.
ISR(  ADC_vect) {       
  if (waitCount) waitCount--; 
  if (timeout) timeout--; 
}

/* OK */
// timer 1 for stepping the motor
ISR( TIM0_COMPA_vect ) { 
 
 // update timer
 OCR0A = rate;
 
 // pulse step line. 
 if (dir == 0)  return;
  
 // set up direction 
 if (dir == 1)  SetBit   ( DIRBIT, PORTB );
 else           ClearBit ( DIRBIT, PORTB );
  
 // do step
 SetBit   ( STEPBIT, PORTB );
 NOP();
 ClearBit ( STEPBIT, PORTB );
  
 // feedback
 fb -= dir; 
  
}





















































