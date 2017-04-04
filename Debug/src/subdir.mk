################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/FastMarching2Star.cpp \
../src/KeypointMatcher.cpp \
../src/SatImg.cpp \
../src/map_align.cpp 

OBJS += \
./src/FastMarching2Star.o \
./src/KeypointMatcher.o \
./src/SatImg.o \
./src/map_align.o 

CPP_DEPS += \
./src/FastMarching2Star.d \
./src/KeypointMatcher.d \
./src/SatImg.d \
./src/map_align.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


