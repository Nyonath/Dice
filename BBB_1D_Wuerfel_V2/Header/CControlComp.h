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
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/SCalib.h"
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/SCalibrated.h"
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
		void calibrate();
		void filter();
		float control();

		int mtmp;
		CBBBHardware mCBBHardware;
		Kalib_Obj mKalib_Obj;
		SMPU6050Data mdata1_raw, mdata2_raw;
		UInt16 mADCVal;
		CalibratedData mCalibratedData;
		SStateVectorData mstate;

		// Winkelschätzung
		float mTau,mT;
		float mrs1; // [m] Abstand Sen sor1 zur Drehachse
		float mrs2; // [m] Abstand Sen sor2 zur Drehachse
		float malpha_acc; // Verhältn is fü r Berechnu ng d es Win kels anh and der Bes ch leunigungs daten
		float mt_Abtast; // [s], Abtas tzeit fü r Berechnu ng des Win kels mit G yroda ten
		float mFilter_Koeff; // [], Koeffizien t fü r Komplementä rfilter
		float malpha_Filter;

		// Kalibrierung
		float Offset_mPhi1_d; // [], Offset d es Gyros en sor1
		float Offset_mPhi2_d; // [], Offset d es Gyros en sor2
		float gyro_gain;
		float gyro_Messbereich; // [rad/s], M es sbereich G yro, zur Umrechnu ng in SI Einheit
		float adc_gain;
		float adc_offset;
		float X1_gain;
		float X1_off;
		float X2_gain;
		float X2_off;
		float Y1_gain;
		float Y1_off;
		float Y2_gain;
		float Y2_off;
		float Phi_C_old;
		float KD1, KD2, KD3;

};

#endif /* HEADER_CCONTROLCOMP_H_ */
