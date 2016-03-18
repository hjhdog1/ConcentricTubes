#include "Section.h"

Section::Section(double sectionLength, double precurvature[3])
{
	this->SetParameter("Length", Parameter(sectionLength));
	this->SetParameter("ux", Parameter(precurvature[0]));
	this->SetParameter("uy", Parameter(precurvature[1]));
	this->SetParameter("uz", Parameter(precurvature[2]));
}