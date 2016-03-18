#include "Tube.h"

Tube::Tube(double bendingStiffness, double PoissonsRatio, std::vector<Section> _sections)
{
	this->SetParameter("kxy", Parameter(bendingStiffness));
	this->SetParameter("nu", Parameter(PoissonsRatio));
	this->SetParameter("Length", Parameter(0.0));
	this->SetParameter("CollarLength", Parameter(17.0));

	if(!_sections.empty())
		this->sections = _sections;
}

std::vector<Section>& Tube::GetSections ()
{
    return this->sections;
}

void Tube::UpdateLength ()
{
	double L = 0;
	for ( std::vector<Section>::iterator it = this->sections.begin(); it != this->sections.end(); it++)
		L += it->GetValue("Length");

	this->SetParameter("Length", Parameter(L));
}

void Tube::AddSection (Section section)
{
	this->sections.push_back(section);
	this->UpdateLength();
}