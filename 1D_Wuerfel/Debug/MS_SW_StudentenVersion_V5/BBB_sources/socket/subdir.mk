################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MS_SW_StudentenVersion_V5/BBB_sources/socket/CServer.cpp 

OBJS += \
./MS_SW_StudentenVersion_V5/BBB_sources/socket/CServer.o 

CPP_DEPS += \
./MS_SW_StudentenVersion_V5/BBB_sources/socket/CServer.d 


# Each subdirectory must supply rules for building sources it contributes
MS_SW_StudentenVersion_V5/BBB_sources/socket/%.o: ../MS_SW_StudentenVersion_V5/BBB_sources/socket/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


