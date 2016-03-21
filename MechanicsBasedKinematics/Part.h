﻿#pragma once

#include <map>
#include <vector>


class Part
{
    protected:
		std::vector<double*> freeParameters;

    public:

		std::vector<double*>& GetFreeParameters ();

		
};
