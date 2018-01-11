/*
 * SCalib.h
 *
 *  Created on: Dec 21, 2017
 *      Author: vmuser
 */

#ifndef MS_SW_STUDENTENVERSION_V5_BBB_SOURCES_LIB_SCALIB_H_
#define MS_SW_STUDENTENVERSION_V5_BBB_SOURCES_LIB_SCALIB_H_

struct Kalib_Obj
{
		float S0x0;
		float S0y0;
		float S1x0;
		float S1y0;
		float Gain_X1;
		float Gain_Y1;
		float Gain_X2;
		float Gain_Y2;
		float gain_phi;
		float S0phi0;
		float S1phi0;
		float gain_psi;
		float S1psi0;
		float phi0;
		float psi0;
		float alpha;
};

#endif /* MS_SW_STUDENTENVERSION_V5_BBB_SOURCES_LIB_SCALIB_H_ */
