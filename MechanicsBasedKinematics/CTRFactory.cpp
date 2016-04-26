﻿#include "CTRFactory.h"

CTRFactory::CTRFactory()
{
}

CTR* const CTRFactory::buildCTR (std::string robotXML)
{
	double nu = 0.3;	// Poisson's ratio

 //   // Tube 1
	//double precurv[3] = {0.0, 1.0/150.0, 0.0};
	//Section sec1oftube1(72.0, precurv);
	//double k1 = 1.0;
	//Tube tube1(k1, nu);
	//tube1.AddSection(sec1oftube1);


	//// Tube 2
	//precurv[1] = 0.0;
	//Section straightSection(17.0,precurv);
	//precurv[1] = 1.0/150.0;
	//double k2 = 1.0/0.955;
	//Section sec2oftube2(72.0,precurv);
	//Tube tube2(k2, nu);
	//tube2.AddSection(straightSection);
	//tube2.AddSection(sec2oftube2);

	//// Tube 3
	//precurv[1] = 0.0;
	//straightSection.sectionLength = 34.0 + 72.0;
	//precurv[1] = 1.0/40;
	//Section sec2oftube3(35, precurv);
	//Tube tube3((k1 + k2)/5.8, nu);
	//double precurv[3] = {0.0, 1.0/150.0, 0.0};
	//Section sec1oftube1(72.0, precurv);
	//double k1 = 1.0;
	//Tube tube1(k1, nu);
	//tube1.AddSection(sec1oftube1);


	double precurv[3] = {0.0, 1.0/265.0, 0.0};
	Section sec1oftube1(150.0, precurv);
	double k1 = 1.0;
	Tube tube1(k1, nu);
	tube1.AddSection(sec1oftube1);

	// Tube 2
	precurv[1] = 0.0;
	Section straightSection(17.0,precurv);
	precurv[1] = 1.0/265.0;
	double k2 = 1.0;
	Section sec2oftube2(150.0,precurv);
	Tube tube2(k2, nu);
	tube2.AddSection(straightSection);
	tube2.AddSection(sec2oftube2);

	// Tube 3
	precurv[1] = 0.0;
	straightSection.sectionLength = 34.0 + 150.0;
	precurv[1] = 1.0/70;
	Section sec2oftube3(87.0, precurv);
	Tube tube3((k1 + k2)/7.0, nu);

	tube3.AddSection(straightSection);
	tube3.AddSection(sec2oftube3);

	CTR* const robot = new CTR();
	robot->AddTube(tube1);
	robot->AddTube(tube2);
	robot->AddTube(tube3);
	robot->Initialize();

	// free parameters - Poisson's ratios of all tubes should be synced.
	for(int i = 0 ; i < 3; ++i)
	{
		if(i != 0)
		{
			robot->freeParameters.push_back(&robot->tubes[i].kxy);
			robot->variances.push_back(1);
		}

		robot->freeParameters.push_back(&robot->tubes[i].sections.back().precurvature[1]);
		robot->variances.push_back(0.1);
	}
	/////////////////////////////////////

	return robot;
}

