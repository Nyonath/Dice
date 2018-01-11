/*
 * CCommComp.h
 *
 *  Created on: Dec 7, 2017
 *      Author: vmuser
 */

#ifndef HEADER_CCOMMCOMP_H_
#define HEADER_CCOMMCOMP_H_
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/IRunnable.h"
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/CContainer.h"
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/SContent.h"
#include "../MS_SW_StudentenVersion_V5/BBB_sources/socket/CServer.h"
#include <iostream>
#include <unistd.h>
using namespace std;

extern CContainer CCont;



class CCommComp : public IRunnable
{
	public:
		CCommComp();
		~CCommComp();
		void run();
		void init();
	private:
		SContent mScont;
		CServer mServer;
		int mtmp=0;
};



#endif /* HEADER_CCOMMCOMP_H_ */
