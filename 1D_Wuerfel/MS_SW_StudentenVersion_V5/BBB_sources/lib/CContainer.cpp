/**
 * @file	CContainer.cpp
 * @author	Michael Meindl
 * @date	5.12.2016
 * @brief	Method definitions for the container.
 */
#include "CContainer.h"

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

	pdata1.mPadding = pint;
	pdata1.mPhi__d = pint;
	pdata1.mX__dd = pint;
	pdata1.mY__dd = pint;
	pdata2.mPadding = pint;
	pdata2.mPhi__d = pint;
	pdata2.mX__dd = pint;
	pdata2.mY__dd = pint;
	pstate.mPhi_A = pfloat;
	pstate.mPhi_C = pfloat;
	pstate.mPhi_G = pfloat;
	pstate.mPhi__d = pfloat;
	pstate.mPsi__d = pfloat;
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
