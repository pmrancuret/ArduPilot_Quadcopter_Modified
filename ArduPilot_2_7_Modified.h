// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef ArduPilot_2_7_Modified_H_
#define ArduPilot_2_7_Modified_H_
#include "Arduino.h"
//add your includes for the project ArduPilot_2_7_Modified here
#include "AP_Config.h"
#include "AP_PID_settings.h"
#include "defines.h"
#include <avr/eeprom.h>

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

// global variable definitions
// GENERAL VARIABLE DECLARATIONS
// --------------------------------------------
extern byte 	control_mode;
extern boolean failsafe;	// did our throttle dip below the failsafe value?
extern boolean ch3_failsafe;
//extern byte 	config_tool_options;		// a bitmask defining config tool options such as altitude hold
extern byte 	crash_timer;
extern int 	egg_dist;

/* Radio values
		Channel assignments
			1	Ailerons (rudder if no ailerons)
			2	Elevator
			3	Throttle
			4	Rudder (if we have ailerons)
*/
extern int16_t radio_min[4];	// may be reset by init sequence
extern int16_t radio_trim[4];							// may be reset by init sequence
extern int16_t radio_max[4];	// may be reset by init sequence

extern int16_t radio_in[4];			// current values from the transmitter - microseconds
extern int16_t radio_out[4];			// PWM to ROLL PITCH Servos
extern float servo_out[4];						// current values to the servos - -45 to 45 degrees, except [3] is 0 to 100

extern int16_t elevon1_trim;
extern int16_t elevon2_trim;
extern int16_t ch1_temp;			// Used for elevon mixing
extern int16_t ch2_temp;
extern int16_t ch3_raw;

extern uint16_t timer_ovf_a;
extern uint16_t timer_ovf_b;
extern uint16_t timer_ovf;

// for elevons radio_in[CH_ROLL] and radio_in[CH_PITCH] are equivalent aileron and elevator, not left and right elevon


/*	PID Control variables
		Cases
		1	Aileron servo control	(rudder if no ailerons)
		2	Elevator servo control
		3	Rudder servo control 	(if we have ailerons)
		4	Roll set-point control
		5	Pitch set-point based on airspeed error control
		6	Pitch set-point based on altitude error control		(if we do not have airspeed sensor)
		7	Throttle based on Energy Height (Total Energy) error control
		8	Throttle based on altitude error control
*/

extern float kp[8];
extern float ki[8];
extern float kd[8];

extern const float integrator_max[8];

extern float	integrator[8];	// PID Integrators
extern float	last_error[8];	// PID last error for derivative
extern float 	nav_gain_scaler;			// Gain scaling for headwind/tailwind


// GPS variables
// -------------
extern byte 	ground_start_count;			// have we achieved first lock and set Home?
extern byte 	GPS_fix;		// This variable store the status of the GPS
extern boolean GPS_light;		// status of the GPS light
extern boolean GPS_new_data;		// fresh data from the GPS needs to be used
extern const float t7;	// used to scale GPS values for EEPROM storage
extern long 	iTOW; 			// GPS Millisecond Time of Week
extern byte	NumSats;			// number of sats
extern boolean imu_ok;		// the IMU is sending data correctly
extern float 	scaleLongUp;			// used to reverse longtitude scaling
extern float 	scaleLongDown;			// used to reverse longtitude scaling


// used to consruct the GPS data from Bytes to ints and longs
// ----------------------------------------------------------
union long_union {
	int32_t dword;
	uint8_t	byte[4];
} ;

extern union long_union longUnion;

union int_union {
	int16_t word;
	uint8_t	byte[2];
};

extern union int_union intUnion;

// Location & Navigation
// ---------------------
extern byte 	wp_radius;			// meters
extern int 	ground_speed;			// m/s * 100
extern long 	ground_course;			// deg * 100 dir of plane
extern long 	hold_course;			// deg * 100 dir of plane
extern long	nav_bearing;			// deg * 100 : 0 to 360 current desired bearing to navigate
extern long 	target_bearing;			// deg * 100 : 0 to 360 location of the plane to the target
extern long 	crosstrack_bearing;			// deg * 100 : 0 to 360 desired angle of plane to target
extern int 	climb_rate;			// m/s * 100
extern byte	loiter_radius; // meters


// Airspeed
// --------
extern float 	airspeed; 							// m/s * 100
extern int		airspeed_cruise;		// m/s * 100 : target airspeed sensor value

// Location Errors
// ---------------
extern long 	bearing_error; 			// deg * 100 : 18000 to -18000
extern long 	altitude_error;			// meters * 100 we are off in altitude
extern float 	airspeed_error;			// m/s * 100:
extern float	crosstrack_error;			// deg * 100 : 18000 to -18000  meters we are off trackline
extern long 	energy_error;

// Sensors
// --------
extern float 	airpressure_raw;		// Airspeed Sensor - is a float to better handle filtering
extern int 	airpressure_offset;		// analog air pressure sensor while still
extern int 	airpressure;		// airspeed as a pressure value
extern float 	battery_voltage;		// Battery Voltage
extern float 	filter_batt_voltage; 		// Filtered Battery Voltage mjc

extern long analog0;		// Thermopiles - Pitch
extern long analog1;		// Thermopiles - Roll
extern long analog2;		// Thermopiles - Z
extern int ir_max;		// used to scale Thermopile output to 511
extern int ir_max_save;		// used to scale Thermopile output to 511
extern long roll_sensor;		// how much we're turning in deg * 100
extern long pitch_sensor;		// our angle of attack in deg * 100


// Performance
// -----------
extern int 	max_altitude;			// meters : read by config tool for seeing how high we've gone
extern byte 	max_speed;			// m/s 	  : read by config tool for seeing how fast we've gone


// flight mode specific
// --------------------
extern boolean takeoff_complete;			// Flag for using take-off controls
extern boolean land_complete;

// Loiter management
// -----------------
extern long 	old_target_bearing;			// deg * 100
extern int		loiter_total;			// deg : how many times to loiter * 360
extern int 	loiter_delta;			// deg : how far we just turned
extern int		loiter_sum;			// deg : how far we have turned around a waypoint


// these are the values for navigation control functions
// ----------------------------------------------------
extern long 	nav_roll;			// deg * 100 : target roll angle
extern long 	nav_pitch;			// deg * 100 : target pitch angle
extern int 	throttle_cruise;	// 0-100 : target throttle output for average speed
extern long	altitude_estimate;			// for smoothing GPS output


// Waypoints
// ---------
extern long 	wp_distance;		// meters - distance between plane and next waypoint
extern long 	wp_totalDistance;		// meters - distance between old and next waypoint
extern byte 	wp_total;		// # of waypoints
extern byte 	wp_index;		// Current WP index, -1 is RTL

// 3D Location vectors
// -------------------
struct Location {
	long lat;				// Lattitude * 10**7
	long lng;				// Longitude * 10**7
	long alt;				// Altitude in centimeters (meters * 100)
};

extern struct Location home;	// home location
extern struct Location prev_WP;	// last waypoint
extern struct Location current_loc;	// current location
extern struct Location next_WP;	// next waypoint
extern long target_altitude;		// used for
extern long offset_altitude;		// used for

// IMU specific
// ------------
#if GPS_PROTOCOL == 3
	extern long perf_mon_timer;
	extern byte imu_health;
	extern int G_Dt_max;						// Max main loop cycle time in milliseconds
	extern byte gyro_sat_count;
	extern byte adc_constraints;
	extern byte renorm_sqrt_count;
	extern byte renorm_blowup_count;
	extern byte gps_payload_error_count;
	extern byte gps_checksum_error_count;
	extern byte gps_pos_fix_count;
	extern byte gps_nav_fix_count;
	extern byte gps_messages_sent;
	extern byte gps_messages_received;
	extern int imu_messages_received;
	extern byte imu_payload_error_count;
	extern byte imu_checksum_error_count;
#endif

// System Timers
// --------------
extern unsigned long fast_loopTimer;		// Time in miliseconds of main control loop
extern unsigned long medium_loopTimer;		// Time in miliseconds of navigation control loop
extern byte medium_loopCounter;		// Counters for branching from main control loop to slower loops
extern byte slow_loopCounter;		//
extern unsigned long deltaMiliSeconds;		// Delta Time in miliseconds
extern unsigned long dTnav;		// Delta Time in milliseconds for navigation computations
extern int mainLoop_count;
extern unsigned long elapsedTime;		// for doing custom events

//add your function definitions for the project ArduPilot_2_7_Modified here

// functions from ArduPilot_2_7_Modified
extern void fast_loop();
extern void medium_loop();
extern void slow_loop();
extern void update_current_flight_mode(void);
extern void update_GPS(void);
extern void update_navigation();

// functions from attitude.pde
extern void calc_nav_pitch();
extern void calc_nav_roll();
extern void calc_throttle();
extern void crash_checker();
extern void stabilize();
extern float PID(long PID_error, long dt, int PID_case);
extern void reset_I(void);

// functions from control_modes.pde
extern void read_control_switch();
extern void reset_control_switch();
extern byte readSwitch(void);
extern void initControlSwitch();

// functions from debug.pde
extern void debug_subsystem();

// functions from EEPROM.pde
extern void set_max_altitude_speed(void);
extern void restore_EEPROM(void);
extern void save_EEPROM_groundstart(void);

// functions from events.pde
extern void mainLoop_event(void);
extern void mediumLoop_event(void);
extern void waypoint_event(byte event);
extern void low_battery_event(void);
extern void startup_air_event();
extern void failsafe_event();
void startup_ground_event();

// functions from GCS_Standard_text.pde
extern void send_message(byte id);
extern void print_attitude(void);
extern void print_control_mode(void);
extern void print_attitude(void);
extern void print_position(void);
extern void printPerfData(void);
extern void send_message(byte id, long param);
extern void print_current_waypoints();

// functions from GPS_IMU.pde
extern void decode_gps(void);
extern void init_gps(void);
extern void checksum(byte data);
extern void GPS_join_data();
extern void IMU_join_data();
extern void IMU2_join_data();
extern int32_t join_4_bytes(byte Buffer[]);
extern void PERF_join_data();

// functions from navigation.pde
extern void calc_altitude_error();
extern void calc_bearing_error();
extern void navigate();
extern void update_crosstrack(void);
extern void wrap_bearing();
extern int get_altitude_above_home(void);
extern long get_bearing(struct Location *loc1, struct Location *loc2);
extern long getDistance(struct Location *loc1, struct Location *loc2);
extern void reset_crosstrack();

// functions from radio.pde
extern void read_radio();
extern void throttle_failsafe(int pwm);
extern void init_radio();
extern void trim_control_surfaces();
void trim_radio();

// functions from sensors.pde
extern void read_XY_sensors();
extern void read_z_sensor(void);
extern long x_axis(void);// roll
extern long y_axis(void);// pitch
extern long getRoll(void);
extern long getPitch(void);

// functions from servos.pde
extern void set_servos_4();
extern void set_ch1_degrees(float deg);
extern void set_ch2_degrees(float deg);
extern void set_ch4_degrees(float deg);
extern void set_servo_mux(boolean mode);
extern void update_throttle();
extern void demo_servos();
extern void init_PWM();

// functions from system.pde
extern void init_ardupilot();
extern void update_GPS_light(void);
extern void set_mode(byte mode);
extern unsigned long freeRAM();
extern void print_launch_params(void);
extern void print_radio();
extern void print_waypoints();
extern void setGPSMux(void);
extern byte startup_check(void);
extern void startup_ground(void);

// functions from timing.pde
extern unsigned long DIYmillis();

// functions from waypoints.pde
extern void init_home();
extern void reached_waypoint();
extern struct Location get_wp_with_index(int i);
extern void load_waypoint();
extern void loiter_at_location();
extern void return_to_launch();
extern void precalc_waypoint_distance(void);
extern long read_alt_to_hold();
extern struct Location set_wp_with_index(struct Location temp, int i);

//Do not add code below this line
#endif /* ArduPilot_2_7_Modified_H_ */
