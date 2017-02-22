#pragma once
#include <vector>
#include "Eigen/Dense"

class TiXmlElement;
class TiXmlNode;

class Point
{
	double x;
	double y;
	double z;
public:
	Point();
	Point(double x, double y, double z);
	~Point();
	
	void GetPointCoords(double& x, double& y, double& z) const;
	void GetPointCoords(double coords[3]) const;
	void GetPointCoords(::std::vector<double>& coords) const;
	void GetPointCoords(::Eigen::Vector3d& coords) const;

	void SetPointCoords(double x, double y, double z);
	void SetPointCoords(const double coords[3]);
	void SetPointCoords(const ::std::vector<double>& coords);
};


class Measurement
{
	::std::vector<double> configuration;
	::std::vector<Point> shape;
	::std::vector<double> arcLength;

public:
	Measurement();
	~Measurement();

	::std::vector<double> GetConfiguration() const;
	void SetConfiguration(const ::std::vector<double>& conf); 

	::std::vector<Point> GetShape() const;
	::std::vector<::Eigen::Vector3d> GetShapeEig() const;

	void SetShape(const ::std::vector<::Eigen::Vector3d>& points);
	void SetShape(const ::std::vector<Point>& shape);

	::std::vector<double> GetArcLength() const;
	void SetArcLength(const ::std::vector<double>& s);

	friend ::std::ostream& operator << (::std::ostream& os, Measurement& meas);

};


typedef ::std::vector<Measurement> ShapeDataset;

void BuildShapeDatasetFromString(const char* filename, ShapeDataset& dataset);

void ShapeDatasetToString(ShapeDataset& dataset, const char* filename);

void AppendMeasurement(TiXmlElement* root, Measurement& meas);


