#include "ArduPilot_2_7_Modified.h"

byte switchPosition 	= 0;
byte oldSwitchPosition 	= 0;

void read_control_switch()
{
	byte switchPosition = readSwitch();
	if (oldSwitchPosition != switchPosition){
		
		switch(switchPosition)
		{
			case 1: // First position
			set_mode(POSITION_1);
			break;
	
			case 2: // middle position
			/*
				if (GPS_fix != FAILED_GPS){
					set_mode(RTL);
				}else{
					set_mode(CIRCLE);
				}
			*/
			set_mode(POSITION_2);
			break;
	
			case 3: // last position
			set_mode(POSITION_3);
			break;
		}

		oldSwitchPosition = switchPosition;

		// reset navigation integrators
		// -------------------------
		reset_I();
	}
}

void reset_control_switch()
{
	oldSwitchPosition = 0;
	read_control_switch();
}

byte readSwitch(void){
	static uint16_t counter = 0;
	static uint16_t laststate = INITIAL_POS;

	if(digitalRead(4) == HIGH){
		if(digitalRead(5) == HIGH){

			if (laststate != 2) counter++;

			// Middle Switch Position
			// ----------------------
			if (counter >= NUMCOUNTS_POS2)
			{
				laststate = 2;
				counter = 0;
				return 2;
			}
			else
			{
				return laststate;
			}

		}else{

			if (laststate != 3) counter++;

			// 3rd Switch Position
			// -------------------
			if (counter >= NUMCOUNTS_POS3)
			{
				laststate = 3;
				counter = 0;
				return 3;
			}
			else
			{
				return laststate;
			}
		}
	}else{

		if (laststate != 1) counter++;

		// 1st Switch Position
		// ----------------------
		if (counter >= NUMCOUNTS_POS1)
		{
			laststate = 1;
			counter = 0;
			return 1;
		}
		else
		{
			return laststate;
		}
	}
}

void initControlSwitch()
{
	oldSwitchPosition = switchPosition = readSwitch();
}
