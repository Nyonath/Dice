################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MS_SW_StudentenVersion_V5/BBB_sources/lib/AComponentBase.cpp \
../MS_SW_StudentenVersion_V5/BBB_sources/lib/Assertion.cpp \
../MS_SW_StudentenVersion_V5/BBB_sources/lib/CBinarySemaphore.cpp \
../MS_SW_StudentenVersion_V5/BBB_sources/lib/CContainer.cpp \
../MS_SW_StudentenVersion_V5/BBB_sources/lib/CMutex.cpp \
../MS_SW_StudentenVersion_V5/BBB_sources/lib/CThread.cpp 

OBJS += \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/AComponentBase.o \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/Assertion.o \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/CBinarySemaphore.o \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/CContainer.o \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/CMutex.o \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/CThread.o 

CPP_DEPS += \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/AComponentBase.d \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/Assertion.d \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/CBinarySemaphore.d \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/CContainer.d \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/CMutex.d \
./MS_SW_StudentenVersion_V5/BBB_sources/lib/CThread.d 


# Each subdirectory must supply rules for building sources it contributes
MS_SW_StudentenVersion_V5/BBB_sources/lib/%.o: ../MS_SW_StudentenVersion_V5/BBB_sources/lib/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


