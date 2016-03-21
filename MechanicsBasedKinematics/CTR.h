﻿#pragma once

#include "Tube.h"

class CTR
{
	friend class CTRFactory;

public:
	CTR();
	~CTR();

    private:
        double* tubeRotation;
		// z-coordinate displacement from global frame to tube base frame.
		// global frame is the base frame of outer most tube in the robot's initial configuration.
        double* tubeTranslation;
        std::vector<Tube> tubes;
        double length;
        double* upperTubeTranslationLimit;
        double* lowerTubeTranslationLimit;
        int numTubes;

    protected:
        void UpdateLength ();
        void ComputeJointLimits ();
		bool CheckJointLimits(const double* translation) const;
        void Initialize ();
        void AddTube (Tube tube);
		bool TubeExists (double s, int tubeID) const;
        
    public:
        bool UpdateConfiguration (const double* rotation, const double* translation);
        bool ComputePrecurvature (double s, int tubeID, const double* precurvature[3]);
		double GetStiffness(int tubeID) {return tubes[tubeID].GetBendingStiffness();};
		double GetPoissonsRatio(int tubeID) {return tubes[tubeID].GetPoissonsRatio();};
		double GetLength() const {return length;};
		int GetNumOfTubes() const {return numTubes;};
		//void GetConfiguration(const double* rot, const double* trans) const {rot = this->tubeRotation; *trans = this->tubeTranslation; };
		//void GetRotation(const double* rot) const {rot = this->tubeRotation;};
		//void GetTranslation (const double* tran) const {tran = this->tubeTranslation;};
		double* GetRotation() const {return this->tubeRotation;};
		double* GetTranslation() const {return this->tubeTranslation;};
        std::vector<Tube>& GetTubes ();

		void GetExistingTubes(const double s, std::vector<int>& tubeIDs) const;



};
