/*
 * CCommComp.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: vmuser
 */

#include "../Header/CCommComp.h"

CCommComp::CCommComp()
{
	//
}

CCommComp::~CCommComp()
{
}
void CCommComp::run()
{
	std::cout << "Communication Comp" << std::endl;
	int i = 0;
	while (1)
	{

			bool ret = CCont.getContent(true, mScont);

			if (ret)
			{
				std::cout << std::endl;
				std::cout << "DATASET:" << i << std::endl;
				std::cout << "ADC Value: " << mScont.mADCValue << std::endl;
				std::cout << "Torque Value: " << mScont.mMotorTorque << std::endl;
				std::cout << "S1Padding Value: " << mScont.mSensor1Data.mPadding << std::endl;
				std::cout << "S1Phi Value: " << mScont.mSensor1Data.mPhi__d << std::endl;
				std::cout << "S1XDD Value: " << mScont.mSensor1Data.mX__dd << std::endl;
				std::cout << "S1YDD Value: " << mScont.mSensor1Data.mY__dd << std::endl;
				std::cout << "S2Padding Value: " << mScont.mSensor2Data.mPadding << std::endl;
				std::cout << "S2Phi Value: " << mScont.mSensor2Data.mPhi__d << std::endl;
				std::cout << "S2XDD Value: " << mScont.mSensor2Data.mX__dd << std::endl;
				std::cout << "S2YDD Value: " << mScont.mSensor2Data.mY__dd << std::endl;
				std::cout << "PHIA Value: " << mScont.mStateData.mPhi_A << std::endl;
				std::cout << "PHIC Value: " << mScont.mStateData.mPhi_C << std::endl;
				std::cout << "PHIG Value: " << mScont.mStateData.mPhi_G << std::endl;
				std::cout << "PHID Value: " << mScont.mStateData.mPhi__d << std::endl;
				std::cout << "PSID Value: " << mScont.mStateData.mPsi__d << std::endl;
				mServer.transmitMessage(mScont);
				i++;

			}

	}
}
void CCommComp::init()
{
	mServer.init();
	mServer.waitForClient();
}

