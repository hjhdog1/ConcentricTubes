// ModelTraining.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Utilities.h"
#include "lwpr.hh"

#include <vector>

int _tmain(int argc, _TCHAR* argv[])
{
	::std::string pathToFile("training.data");
	::std::vector<::std::string> linesStr = ReadLinesFromFile(pathToFile);

	::std::string modelNameStr = GetDateString() + "_lwpr_model.bin";

	LWPR_Object model(3,6);
	model.setInitD(40);

	for (::std::vector<::std::string>::iterator it = linesStr.begin(); it != linesStr.end(); ++it)
	{
		doubleVec dataVec = DoubleVectorFromString(*it);
		doubleVec dataInput(dataVec.data(), dataVec.data() + 3);
		doubleVec dataOutput(dataVec.data() + 3, dataVec.data() + 9);
		model.update(dataInput, dataOutput);
	}

	model.writeBinary(modelNameStr.c_str());

	return 0;
}

