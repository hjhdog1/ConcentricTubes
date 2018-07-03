#pragma once

#include "CTR.h"

class CTRFactory
{
    public:
		CTRFactory();
        static CTR* const buildCTR (std::string robotXML);
		static CTR* const buildBalancedPair(::std::string robotXML);
};
