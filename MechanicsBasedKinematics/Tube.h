#pragma once

#include "Section.h"

class Tube :  public Part
{
friend class CTRFactory;

std::vector<Section> sections;

public:
	Tube(double bendingStiffness, double PoissonsRatio, std::vector<Section> sections = std::vector<Section>());
	std::vector<Section>& GetSections ();

protected:
    void UpdateLength ();

private:
	void AddSection (Section section);

};
