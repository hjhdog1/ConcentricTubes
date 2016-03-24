#pragma once



class Filter
{
protected:
	double rot[9], pos[3];

public:
	Filter();
	virtual ~Filter();
	virtual bool StepFilter(double _rot[9], double _pos[3]);
	virtual void Initialize() = 0;
	
protected:
	virtual void predict() = 0;
	virtual bool update() = 0;
	
	virtual void updateMeasurement(double _rot[9], double _pos[3]);


};
