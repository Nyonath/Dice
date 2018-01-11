/*
 * main.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: Robin Tenscher
 *      		Tim Schanz
 */



// Standard includes
#include <iostream>

//Project includes
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/CContainer.h"
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/Global.h"
#include "../MS_SW_StudentenVersion_V5/BBB_sources/lib/CThread.h"
#include "../Header/CControlComp.h"
#include "../Header/CCommComp.h"


CContainer CCont;

int main()
{
	CCommComp CommObj;
	CControlComp ControlObj;

	CThread CommTh=CThread(&CommObj, CThread::PRIORITY_NORM);
	CThread ControlTh=CThread(&ControlObj, CThread::PRIORITY_NORM);

	CommTh.start();
	ControlTh.start();


	CommTh.join();
	ControlTh.join();

	return 0;
}




