/*
 * CControlComp.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: vmuser
 */

#include "../Header/CControlComp.h"


CControlComp::CControlComp()
{
	mtmp = 0;
}

CControlComp::~CControlComp()
{
}
void CControlComp::run()
{

	std::cout << "Control Comp " << std::endl;
	UInt16 adcVal = 0;
	float f = 0.0;
	time_t time_1,time_2;
	SMPU6050Data data1, data2;
	SStateVectorData state;
	while (1)
	{
		time(&time_1);
		mCBBHardware.fetchValues(adcVal, data1, data2);		//Collect data from sensors

		CCont.setContent(adcVal, f, data1, data2, state);		//Write data in Container

		f += 0.1;
		CCont.signalReader();
		time(&time_2);
		usleep(20000-(time_2-time_1));
	}

}
void CControlComp::init()
{

}

void CControlComp::Turningpoint(SMPU6050Data &pdata1, SMPU6050Data &pdata2, SStateVectorData &pstate)
{

	UInt16 Phi_A =atan(((pdata1.mX__dd - (alpha_acc* pdata2.mX__dd))/(pdata1.mY__dd - (alpha_acc* pdata2.mY__dd)))*(-1.0));
	pstate.mPhi_A= Phi_A;
	//pdata1.
	//pstate.
}

void CControlComp::Filter(SMPU6050Data &pdata1, SMPU6050Data &pdata2, SStateVectorData &pstate)
{
	//pstate.
}



