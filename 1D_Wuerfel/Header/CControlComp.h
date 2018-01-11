/*
 * CControlComp.h
 *
 *  Created on: Dec 7, 2017
 *      Author: vmuser
 */

#ifndef HEADER_CCONTROLCOMP_H_
#define HEADER_CCONTROLCOMP_H_
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/IRunnable.h"
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/CContainer.h"
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/Global.h"
#include "../MS_SW_StudentenVersion_V5/BBB_sources/Hardware/CBBBHardware.h"
#include <math.h>

#include <iostream>
#include <unistd.h>

extern CContainer CCont;

class CControlComp: public IRunnable
{
public:
	CControlComp();
	~CControlComp();
	void run();
	void init();
private:
	void Filter(SMPU6050Data &pdata1, SMPU6050Data &pdata2, SStateVectorData &pstate);
	void Turningpoint(SMPU6050Data &pdata1, SMPU6050Data &pdata2, SStateVectorData &pstate);
	int mtmp;
	CBBBHardware mCBBHardware;
	const UInt16 mTau1 = 1;
	const UInt16 mTau2 = 1;
	const float rs1 = 0.14; // [m] Abstand Sen sor1 zur Drehachse
	const float rs2 = 0.061; // [m] Abstand Sen sor2 zur Drehachse
	const float alpha_acc = rs1 / rs2; // Verhältn is fü r Berechnu ng d es Win kels anh and der Bes ch leunigungs daten
	const float t_Abtast = 0.02; // [s], Abtas tzeit fü r Berechnu ng des Win kels mit G yroda ten
	const float Offset_acc = -0.1719; // [rad], Offset fü r Win kelberechnu ng mit "T rick"
	const float Offset_mPhi1_d = -3.703551912568306; // [], Offset d es Gyros en sor1
	const float Offset_mPhi2_d = -26.290983606557376; // [], Offset d es Gyros en sor2
	const float gyro_Messbereich = 69.81317; // [rad/s], M es sbereich G yro, zur Umrechnu ng in SI Einheit
	const float Filter_Koeff = 0.98; // [], Koeffizien t fü r Komplementä rfilter

};

#endif /* HEADER_CCONTROLCOMP_H_ */
