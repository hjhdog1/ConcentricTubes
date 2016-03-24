#include "Filter.h"
#include <string.h>

Filter::Filter()
{
}

Filter::~Filter()
{
}

bool Filter::StepFilter(double _rot[9], double _pos[3])
{
	this->updateMeasurement(_rot,_pos);

	this->predict();
	return this->update();
}

void Filter::updateMeasurement(double _rot[9], double _pos[3])
{
	memcpy(this->rot, _rot, sizeof(double)*9);
	memcpy(this->pos, _pos, sizeof(double)*3);
}

