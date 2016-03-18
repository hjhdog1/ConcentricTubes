#pragma once

#include "Parameter.h"
#include <map>
#include <vector>

typedef std::map<std::string, Parameter> ParameterMap;

class Part
{
    protected:
        ParameterMap parameters;

    public:
        void SetParameter (const std::string& key, const Parameter& value);
        void GetParameter (const std::string& key, Parameter& value) const;
		double GetValue(const std::string& key) const;

        std::vector<Parameter*> GetFreeParameters ();

		
};
