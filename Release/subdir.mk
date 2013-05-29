################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ArduPilot_2_7_Modified.cpp 

PDE_SRCS += \
../EEPROM.pde \
../GCS_Standard_text.pde \
../GPS_IMU.pde \
../attitude.pde \
../control_modes.pde \
../debug.pde \
../events.pde \
../navigation.pde \
../radio.pde \
../sensors.pde \
../servos.pde \
../system.pde \
../timing.pde \
../waypoints.pde 

OBJS += \
./ArduPilot_2_7_Modified.o \
./EEPROM.o \
./GCS_Standard_text.o \
./GPS_IMU.o \
./attitude.o \
./control_modes.o \
./debug.o \
./events.o \
./navigation.o \
./radio.o \
./sensors.o \
./servos.o \
./system.o \
./timing.o \
./waypoints.o 

CPP_DEPS += \
./ArduPilot_2_7_Modified.d 

PDE_DEPS += \
./EEPROM.d \
./GCS_Standard_text.d \
./GPS_IMU.d \
./attitude.d \
./control_modes.d \
./debug.d \
./events.d \
./navigation.d \
./radio.d \
./sensors.d \
./servos.d \
./system.d \
./timing.d \
./waypoints.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"C:\arduino-1.0.4-windows\arduino-1.0.4\hardware\arduino\cores\arduino" -I"C:\arduino-1.0.4-windows\arduino-1.0.4\hardware\arduino\variants\standard" -I"D:\Documents\Arduino\ArduPilot_2_7_Modified" -I"C:\arduino-1.0.4-windows\arduino-1.0.4\hardware\tools\avr\avr\include" -D__IN_ECLIPSE__=1 -DARDUINO=104 -DUSB_PID= -DUSB_VID= -Wall -Os -ffunction-sections -fdata-sections -fno-exceptions -g -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -c -o "$@" -x c++ "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.pde
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"C:\arduino-1.0.4-windows\arduino-1.0.4\hardware\arduino\cores\arduino" -I"C:\arduino-1.0.4-windows\arduino-1.0.4\hardware\arduino\variants\standard" -I"D:\Documents\Arduino\ArduPilot_2_7_Modified" -I"C:\arduino-1.0.4-windows\arduino-1.0.4\hardware\tools\avr\avr\include" -D__IN_ECLIPSE__=1 -DARDUINO=104 -DUSB_PID= -DUSB_VID= -Wall -Os -ffunction-sections -fdata-sections -fno-exceptions -g -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -c -o "$@" -x c++ "$<"
	@echo 'Finished building: $<'
	@echo ' '


