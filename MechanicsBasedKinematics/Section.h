#pragma once

#include "Part.h"

class Section :  public Part
{
friend class CTRFactory;

public:
	Section(double sectionLength, double precurvature[3]);
};
