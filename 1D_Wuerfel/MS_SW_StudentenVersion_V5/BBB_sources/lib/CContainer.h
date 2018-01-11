/**
 * @author	Michael Meindl
 * @date	5.12.2016
 * @brief	Container class used for communication between the components.
 * 			The comm-component waits for the read semaphore, reads the values
 * 			and transmits them via the TCP/IP-socket.
 * 			The control-component writes the values - without signaling - and gives the read semaphore.
 */
#ifndef CCONTAINER_H
#define CCONTAINER_H
#include "SContent.h"
#include "CBinarySemaphore.h"
#include "../lib/Global.h"

class CContainer
{
public:
	bool getContent(bool waitForever,
					SContent& content);
	void setContent(UInt16 pint, float pfloat, SMPU6050Data &pdata1, SMPU6050Data &pdata2, SStateVectorData &pstate);
	void signalReader();
	bool writeADCValue(const UInt16 adcValue);
	bool writeTorqueValue(const float torque);
	bool writeSensor1Data(const SMPU6050Data& sensorData);
	bool writeSensor2Data(const SMPU6050Data& sensorData);
	bool writeStateData(const SStateVectorData& sensorData);
public:
	CContainer();
private:
	SContent mContent;
	CBinarySemaphore mReadSem;
};

#endif
