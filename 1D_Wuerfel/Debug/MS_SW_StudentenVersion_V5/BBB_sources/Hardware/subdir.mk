################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CADC.cpp \
../MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CBBBHardware.cpp \
../MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CBrake.cpp \
../MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CMPU6050.cpp \
../MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CMotor.cpp 

OBJS += \
./MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CADC.o \
./MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CBBBHardware.o \
./MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CBrake.o \
./MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CMPU6050.o \
./MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CMotor.o 

CPP_DEPS += \
./MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CADC.d \
./MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CBBBHardware.d \
./MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CBrake.d \
./MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CMPU6050.d \
./MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CMotor.d 


# Each subdirectory must supply rules for building sources it contributes
MS_SW_StudentenVersion_V5/BBB_sources/Hardware/%.o: ../MS_SW_StudentenVersion_V5/BBB_sources/Hardware/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


