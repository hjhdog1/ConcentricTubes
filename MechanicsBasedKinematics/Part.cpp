#include "Part.h"

void Part::SetParameter (const std::string& key, const Parameter& value)
{
    parameters[key] = value;
}

void Part::GetParameter (const std::string& key, Parameter& value) const
{
	value = parameters.find(key)->second;
}

double Part::GetValue(const std::string& key) const
{
	return parameters.find(key)->second.value;
}

std::vector<Parameter*> Part::GetFreeParameters ()
{
	std::vector<Parameter*> parameterVector;

	for (ParameterMap::iterator it = parameters.begin(); it != parameters.end(); it++)
	{
		if(it->second.isFree)
			parameterVector.push_back(&(it->second));
	}

	return parameterVector;
}

