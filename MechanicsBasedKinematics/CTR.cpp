#include "CTR.h"

#define MAX_TRANSLATION 100000000000000000

CTR::CTR(): numTubes(0), length(0.0)
{
}

CTR::~CTR()
{
	delete tubeRotation, tubeTranslation, upperTubeTranslationLimit, lowerTubeTranslationLimit;
}

void CTR::UpdateLength ()
{
	length = (--tubes.end())->GetValue("Length") - (tubeTranslation[0] - tubeTranslation[numTubes-1]);
}

void CTR::ComputeJointLimits ()
{
	double accCollarLength = this->tubes[0].GetValue("CollarLength");
	for(int i = 1; i < this->numTubes; ++i)
	{	
		this->upperTubeTranslationLimit[i] = -accCollarLength + this->tubeTranslation[0];
		this->lowerTubeTranslationLimit[i] = this->tubes[0].GetValue("Length") - this->tubes[i].GetValue("Length") + this->tubeTranslation[0];

		accCollarLength += this->tubes[i].GetValue("CollarLength");
	}
}

bool CTR::CheckJointLimits(const double* translation) const
{
	for( int i = 0; i < this->numTubes; ++i)
		if(translation[i] < this->lowerTubeTranslationLimit[i] || translation[i] > this->upperTubeTranslationLimit[i])
			return false;

	return true;
}

// After initialization, stop adding tubes.
void CTR::Initialize ()
{
	this->tubeRotation = new double[numTubes];
	this->tubeTranslation = new double[numTubes];
	for( int i = 0; i < numTubes; ++i)
		this->tubeRotation[i] = this->tubeTranslation[i] = 0.0;
	

	this->upperTubeTranslationLimit = new double[numTubes];
	this->lowerTubeTranslationLimit = new double[numTubes];

	upperTubeTranslationLimit[0] = MAX_TRANSLATION;
	lowerTubeTranslationLimit[0] = -MAX_TRANSLATION;

    this->ComputeJointLimits();
}

bool CTR::TubeExists (double s, int tubeID) const
{
	if( s < 0)
		return false;

	if( s > tubes[tubeID].GetValue("Length") - (tubeTranslation[0] - tubeTranslation[tubeID]) )
		return false;

	return true;
}

bool CTR::UpdateConfiguration (const double* rotation, const double* translation)
{
	if(!this->CheckJointLimits(translation))
		return false;

	memcpy(this->tubeRotation, rotation, sizeof(double)*this->numTubes);
	memcpy(this->tubeTranslation, translation, sizeof(double)*this->numTubes);

	this->UpdateLength();

	return true;
}

void CTR::AddTube (Tube tube)
{
    this->numTubes++;
	this->tubes.push_back(tube);
}

bool CTR::ComputePrecurvature (double s, int tubeID, double* precurvature)
{
	if(!this->TubeExists(s, tubeID))
		return false;

    std::vector<Section>& sections = this->tubes[tubeID].GetSections();
	 
	double accSectionLength = 0;
	double relativeTrans = (this->tubeTranslation[0] - this->tubeTranslation[tubeID]);
	for(std::vector<Section>::iterator it = sections.begin(); it != sections.end(); ++it)
	{
		accSectionLength += it->GetValue("Length");
		if(s + relativeTrans <= accSectionLength)
		{
			precurvature[0] = it->GetValue("ux");
			precurvature[1] = it->GetValue("uy");
			precurvature[2] = it->GetValue("uz");
			return true;
		} 
	}

	return false;
}

std::vector<Tube>& CTR::GetTubes ()
{
    return this->tubes;
}

void CTR::GetExistingTubes(const double s, std::vector<int>& tubeIDs) const
{
	tubeIDs.clear();
	for(int i = 0; i < this->numTubes; ++i)
		if(this->TubeExists (s, i))
			tubeIDs.push_back(i);
}