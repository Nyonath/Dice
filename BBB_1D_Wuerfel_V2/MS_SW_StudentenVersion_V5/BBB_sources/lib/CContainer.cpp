/**
 * @file	CContainer.cpp
 * @author	Michael Meindl
 * @date	5.12.2016
 * @brief	Method definitions for the container.
 */
#include "CContainer.h"
#include <iostream>

CContainer::CContainer() :
		mReadSem(false, false)
{

}
bool CContainer::getContent(bool waitForever, SContent& content)
{
	if (mReadSem.take(waitForever))
	{
		content = mContent;
		return true;
	}
	return false;
}
void CContainer::setContent(UInt16 pint, float pfloat, SMPU6050Data &pdata1, SMPU6050Data &pdata2, SStateVectorData &pstate)
{
	//signalReader();
	writeADCValue(pint);
	writeTorqueValue(pfloat);
	writeSensor1Data(pdata1);
	writeSensor2Data(pdata2);
	writeStateData(pstate);
}
void CContainer::signalReader()
{
	mReadSem.give();
}
bool CContainer::writeADCValue(const UInt16 adcValue)
{
	mContent.mADCValue = adcValue;
	return true;
}
bool CContainer::writeTorqueValue(const float torque)
{
	mContent.mMotorTorque = torque;
	return true;
}
bool CContainer::writeSensor1Data(const SMPU6050Data& sensorData)
{
	mContent.mSensor1Data = sensorData;
	return true;
}
bool CContainer::writeSensor2Data(const SMPU6050Data& sensorData)
{
	mContent.mSensor2Data = sensorData;
	return true;
}
bool CContainer::writeStateData(const SStateVectorData& stateValue)
{
	mContent.mStateData = stateValue;
	return true;
}
