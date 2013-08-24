#include "ArduPilot_2_7_Modified.h"

// swing the servos around to show them we're alive
// ------------------------------------------------
void demo_servos()
{
	delay(30);
	set_servo_mux(true);
	OCR1A = 1600 * 2;
	OCR1B = 1600 * 2;
	delay(400);
	OCR1A = 1400 * 2;
	OCR1B = 1400 * 2;
	delay(200);
	OCR1A = 1500 * 2;
	OCR1B = 1500 * 2;
	set_servo_mux(false);
	delay(30);
}

void set_pwms_and_mux_quad()
{
	set_servo_mux(true);
	OCR1A = 1100 * 2;
	OCR1B = 1100 * 2;
    uint16_t timer_out 	= 1100 % 512;
    timer_ovf_b 		= 1100 / 512;
    timer_ovf_a 		= 1100 / 512;
    timer_out >>= 1;
    OCR2A = timer_out;
    OCR2B = timer_out;
	delay(30);
	set_servo_mux(true);
}

void set_servo_mux(boolean mode)
{
	while(TCNT1 < 20000){};
	if (mode){
		//take over the MUX
		pinMode(4, OUTPUT);
		digitalWrite(4, HIGH);
	}else{
		//release the MUX to allow Manual Control
		digitalWrite(4, LOW); 
		pinMode(4, INPUT);
	}
}
// wants +- 45°
void set_servos_4()
{

 #if QUAD_COPTER==1
  int16_t yaw_differential;
  int16_t roll_differential;
  int16_t pitch_differential;

  /* 	start calculations with all four motors equal to throttle channel input.
   * 	Note that while servo_out is a floating point value, the CH_THROTTLE element
   *  holds an integer value for throttle command (generally between 1000 and 2000).
   *
   *  The other three elements of servo_out contain floating point numbers between
   *  -45 and 45, corresponding to "servo" positions for roll, pitch, and yaw.
   *
   *  In contrast, the quadmot_out[] array simply contains the actual integer throttle
   *  commands going to each of the four motors.  These are generally limited between
   *  1000 and 2000.
   */
  quadmot_out[MOTOR0] = servo_out[CH_THROTTLE];	// Begin with motor 0 set equal to throttle command
  quadmot_out[MOTOR1] = servo_out[CH_THROTTLE];	// Begin with motor 1 set equal to throttle command
  quadmot_out[MOTOR2] = servo_out[CH_THROTTLE];	// Begin with motor 2 set equal to throttle command
  quadmot_out[MOTOR3] = servo_out[CH_THROTTLE];	// Begin with motor 3 set equal to throttle command

  // first apply yaw differential to motor commands
  yaw_differential = (int16_t)(servo_out[CH_RUDDER] * QUADYAW_GAIN);
  yaw_differential=0; // When testing on stand
  quadmot_out[MOTOR0] -= yaw_differential;
  quadmot_out[MOTOR1] += yaw_differential;
  quadmot_out[MOTOR2] -= yaw_differential;
  quadmot_out[MOTOR3] += yaw_differential;

  // next apply roll differential to motor commands
 #if (QUADCONFIG_X == 0)	// if the quadrotor is in a '+' configuration
  roll_differential = (int16_t)(servo_out[CH_ROLL] * QUADPITCHROLL_GAIN);
  quadmot_out[MOTOR3] += roll_differential;
  quadmot_out[MOTOR1] -= roll_differential;
 #else					// if the quadrotor is in an 'X' configuration
  roll_differential = ((int16_t)(servo_out[CH_ROLL] * QUADPITCHROLL_GAIN))>>1;
  quadmot_out[MOTOR0] += roll_differential;
  quadmot_out[MOTOR3] += roll_differential;
  quadmot_out[MOTOR1] -= roll_differential;
  quadmot_out[MOTOR2] -= roll_differential;
 #endif

  // next apply pitch differential to motor commands
 #if (QUADCONFIG_X == 0)	// if the quadrotor is in a '+' configuration
  pitch_differential = (int16_t)(servo_out[CH_PITCH] * QUADPITCHROLL_GAIN);
  quadmot_out[MOTOR0] += pitch_differential;
  quadmot_out[MOTOR2] -= pitch_differential;
 #else					// if the quadrotor is in an 'X' configuration
  pitch_differential = ((int16_t)(servo_out[CH_PITCH] * QUADPITCHROLL_GAIN))>>1;
  pitch_differential=0; // When testing roll channel on stand
  quadmot_out[MOTOR0] += pitch_differential;
  quadmot_out[MOTOR1] += pitch_differential;
  quadmot_out[MOTOR2] -= pitch_differential;
  quadmot_out[MOTOR3] -= pitch_differential;
 #endif

if (radio_in[CH_THROTTLE] < MIN_SPIN_PWM)
{
  // last constrain the calculated motor commands to remain between 1000 and 2000
  quadmot_out[MOTOR0] = 1000;
  quadmot_out[MOTOR1] = 1000;
  quadmot_out[MOTOR2] = 1000;
  quadmot_out[MOTOR3] = 1000;
}
else
{
  // last constrain the calculated motor commands to remain between 1000 and 2000
  quadmot_out[MOTOR0] = constrain(quadmot_out[MOTOR0],MIN_SPIN_PWM,2000);
  quadmot_out[MOTOR1] = constrain(quadmot_out[MOTOR1],MIN_SPIN_PWM,2000);
  quadmot_out[MOTOR2] = constrain(quadmot_out[MOTOR2],MIN_SPIN_PWM,2000);
  quadmot_out[MOTOR3] = constrain(quadmot_out[MOTOR3],MIN_SPIN_PWM,2000);
}

 #if DEBUG_INFLIGHT == 1
  Serial.print("  RollDiffPwm: ");
  Serial.print(roll_differential,DEC); 
  Serial.print("  LeftMotor: ");
  Serial.print(quadmot_out[MOTOR0],DEC);
  Serial.print("  RightMotor: ");
  Serial.println(quadmot_out[MOTOR1],DEC);
 #endif


  /*
   * OCR1A compare value corresponds to Motor 0 output
   * OCR1B compare value corresponds to Motor 1 output
   * OCR2A compare value corresponds to Motor 2 output
   * OCR2B compare value corresponds to Motor 3 output
   */
  // first set Motor 0 pwm output compare
  OCR1A=quadmot_out[MOTOR0]<<1;	// <<1 is a faster way to say *2 for integer

  // next set Motor 1 pwm output compare
  OCR1B=quadmot_out[MOTOR1]<<1;	// <<1 is a faster way to say *2 for integer
   
  // next set Motor 2 pwm output compare - this low resolution timer will overflow a certain number of times before counting the correct amount
  uint16_t timer_out_a 	= quadmot_out[MOTOR2] % 512;	// calculate the compare value -> remainder after correct number of overflows
  timer_ovf_a 		= quadmot_out[MOTOR2] / 512;			// calculate the correct number of overflows to allow
  timer_out_a >>= 1;										// divide compare value by 2
  OCR2A = timer_out_a;										// set compare value

  // last set Motor 3 pwm output compare - this low resolution timer will overflow a certain number of times before counting the correct amount
  uint16_t timer_out_b 	= quadmot_out[MOTOR3] % 512;	// calculate the compare value -> remainder after correct number of overflows
  timer_ovf_b 		= quadmot_out[MOTOR3] / 512;			// calculate the correct number of overflows to allow
  timer_out_b >>= 1;										// divide compare value by 2
  OCR2B = timer_out_b;										// set compare value

 #else

	#if GPS_PROTOCOL == 3
		if(imu_ok == false && control_mode > MANUAL){	        //  We have lost the IMU - Big trouble
			servo_out[CH_ROLL] = 0;   							//  If we have lost imu we will probably crash.  
			servo_out[CH_PITCH] = 0;							//  Neutralize controls, throttle off
			servo_out[CH_THROTTLE] = 0;
		}   
	#endif

	#if MIXING_MODE == 0
		set_ch1_degrees(servo_out[CH_ROLL]); // 45 ° = right turn (unless reversed)
		set_ch2_degrees(servo_out[CH_PITCH]);
	#endif
	
	  /*Elevon mode*/ // 
	#if MIXING_MODE == 1
		set_ch1_degrees(REVERSE_ELEVONS * (servo_out[CH_PITCH] - servo_out[CH_ROLL]));	
		set_ch2_degrees(servo_out[CH_PITCH] + servo_out[CH_ROLL]);
	#endif

	set_ch4_degrees(servo_out[CH_RUDDER]);
	update_throttle();
#endif      
}



// requires +- 45°
void set_ch1_degrees(float deg){

#if MIXING_MODE == 0
	radio_out[CH_ROLL] = radio_trim[CH_ROLL] + ((float)REVERSE_ROLL * deg * 11.111f);
#endif

#if MIXING_MODE == 1
	radio_out[CH_ROLL] =  elevon1_trim + ((float)REVERSE_CH1_ELEVON  * deg * 11.111f);    //
#endif
	radio_out[CH_ROLL] = constrain(radio_out[CH_ROLL], 	radio_min[CH_ROLL], 	radio_max[CH_ROLL]);
	radio_out[CH_ROLL] = constrain(radio_out[CH_ROLL], 	1000, 	2000);
	OCR1A = radio_out[CH_ROLL] * 2;	//OCR1A is the channel 1 pulse width in half microseconds
}


void set_ch2_degrees(float deg){

#if MIXING_MODE == 0
	radio_out[CH_PITCH] = radio_trim[CH_PITCH] + ((float)REVERSE_PITCH * deg * 11.111f);
#endif

#if MIXING_MODE == 1
	radio_out[CH_PITCH] =  elevon2_trim + ((float)REVERSE_CH2_ELEVON * deg * 11.111f);
#endif

	radio_out[CH_PITCH] = constrain(radio_out[CH_PITCH], 	radio_min[CH_PITCH], 	radio_max[CH_PITCH]);
	radio_out[CH_PITCH] = constrain(radio_out[CH_PITCH], 	1000, 	2000);
	OCR1B = radio_out[CH_PITCH] * 2;
}

void set_ch4_degrees(float deg){

	//Serial.print("tdeg:");
	//Serial.print(deg,DEC);
	deg = constrain(deg, -45, 45);
	radio_out[CH_RUDDER] = radio_trim[CH_RUDDER] + ((float)REVERSE_RUDDER * deg * 11.111f);
	//Serial.print("\tradio_out: ");
	//Serial.print(radio_out[CH_RUDDER],DEC);
	radio_out[CH_RUDDER] = constrain(radio_out[CH_RUDDER], 	radio_min[CH_RUDDER], 	radio_max[CH_RUDDER]);
	//Serial.print("\tradio_out: ");
	//Serial.print(radio_out[CH_RUDDER],DEC);
	//Serial.print(" : ");
	
	uint16_t timer_out 	= radio_out[CH_RUDDER] % 512; 
	timer_ovf_b 		= radio_out[CH_RUDDER] / 512;
	timer_out >>= 1;

	if(timer_out != OCR2B)
		OCR2B = timer_out;
}

void no_throttle()
{
	//OCR2A = ch3_timer_min;
}

// sets the throttle timer value based on throttle percent
// -------------------------------------------------------
void update_throttle()
{
	#if THROTTLE_OUT == 1
		// convert 0 to 100% into PWM
		servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], 0, 100);	
		radio_out[CH_THROTTLE] = (servo_out[CH_THROTTLE] * (radio_max[CH_THROTTLE] - radio_min[CH_THROTTLE])) / 100;
		radio_out[CH_THROTTLE] += radio_min[CH_THROTTLE];
	#else	
		radio_out[CH_THROTTLE] = radio_min[CH_THROTTLE];
	#endif
	
	// Jason's fancy 2µs hack
	uint16_t timer_out 	= radio_out[CH_THROTTLE] % 512; 
	timer_ovf_a 		= radio_out[CH_THROTTLE] / 512;
	timer_out >>= 1;
	
	if(OCR2A != timer_out)
		OCR2A = timer_out;
}

// Throttle Timer Interrupt
// ------------------------
ISR(TIMER1_CAPT_vect) // Timer/Counter1 Capture Event
{
	//This is a timer 1 interrupts, executed every 20us 
	PORTB |= B00000001; //Putting the pin high!
	PORTC |= B00010000; //Putting the pin high!	
	TCNT2 = 0; //restarting the counter of timer 2
	timer_ovf = 0;
}

ISR(TIMER2_OVF_vect)
{
	timer_ovf++;
}

ISR(TIMER2_COMPA_vect) // Timer/Counter2 Compare Match A
{
	if(timer_ovf == timer_ovf_a){
		PORTB &= B11111110; //Putting the pin low
	}
}

ISR(TIMER2_COMPB_vect) // Timer/Counter2 Compare Match B Rudder Servo
{
	if(timer_ovf == timer_ovf_b){
		PORTC &= B11101111; //Putting the pin low!
	}
} 


void init_PWM()
{
	// Servo setup
	// -----------
	
	// Timer 1
	TCCR1A = ((1<<WGM11) | (1<<COM1B1) | (1<<COM1A1)); //Fast PWM: ICR1=TOP, OCR1x=BOTTOM,TOV1=TOP
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11); // Clock scaler = 8, 2,000,000 counts per second
	OCR1A = 3000;	// Rudder  - multiply your value * 2; for example 3000 = 1500 = 45°; 4000 = 2000 = 90°
	OCR1B = 3000; 	// Elevator
	ICR1 = 40000; 	//50hz freq...Datasheet says	(system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz = 40000,


	// enable pin change interrupt 2 - PCINT23..16
	PCICR = _BV(PCIE2);
	
	// enable pin change interrupt 0 -  PCINT7..0
	PCICR |= _BV(PCIE0);

	// Throttle;
	// Setting up the Timer 2 - 8 bit timer
	TCCR2A 	= 0x0; //Normal Mode            
	//TCCR2B 	= _BV(CS22); //prescaler 64, at 16mhz (64/16) = 4, the counter will increment 1 every 4us
	TCCR2B 	= _BV(CS21) |_BV(CS20); //prescaler 32, at 16mhz (32/16) = 2, the counter will increment 1 every 2us
	//OCR2A 	= (CH3_MIN-1000) / 4;
	//OCR2B  	= 125; // center the rudder
	servo_out[CH_THROTTLE] = 0;
	update_throttle();
	set_ch4_degrees(0);
	TIMSK1 |= _BV(ICIE1); 	// Timer/Counter1, Input Capture Interrupt Enable //PB0 - output throttle
	TIMSK2 	= _BV(TOIE1) | _BV(OCIE2A) | _BV(OCIE2B);	// Timer/Counter2 Compare Match A		
}
