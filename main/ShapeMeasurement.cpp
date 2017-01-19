#include "ShapeMeasurement.h"
#include <iostream>
#include "tinyxml.h"
#include "Utilities.h"

Point::Point():
	x(0), y(0), z(0)
{
}

Point::Point(double x, double y, double z):
	x(x), y(y), z(z)
{
}

Point::~Point()
{
}
	
void 
Point::GetPointCoords(double& x, double& y, double& z) const
{
	x = this->x;
	y = this->y;
	z = this->z;
}

void 
Point::GetPointCoords(double coords[3]) const
{
	coords[0] = this->x;
	coords[1] = this->y;
	coords[2] = this->z;
}

void 
Point::GetPointCoords(::std::vector<double>& coords) const
{
	coords.resize(0);
	coords.push_back(this->x);
	coords.push_back(this->y);
	coords.push_back(this->z);
}

void 
Point::GetPointCoords(::Eigen::Vector3d& coords) const
{
	coords[0] = this->x;
	coords[1] = this->y;
	coords[2] = this->z;
}

void 
Point::SetPointCoords(double x, double y, double z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void 
Point::SetPointCoords(const double coords[3])
{
	this->x = coords[0];
	this->y = coords[1];
	this->z = coords[2];
}

void 
Point::SetPointCoords(const ::std::vector<double>& coords)
{
	this->SetPointCoords(coords.data());
}

Measurement::Measurement():
	configuration(0), shape(0), arcLength(0)
{
}

Measurement::~Measurement()
{
}

::std::vector<double>
Measurement::GetConfiguration() const
{
	return this->configuration;
}

void 
Measurement::SetConfiguration(const ::std::vector<double>& conf)
{
	this->configuration = conf;
}

::std::vector<Point>
Measurement::GetShape() const
{
	return this->shape;
}

::std::vector<::Eigen::Vector3d> 
Measurement::GetShapeEig() const
{
	::Eigen::Vector3d tmp;
	::std::vector<::Eigen::Vector3d> pointsEig;
	for(::std::vector<Point>::const_iterator it = this->shape.begin(); it != this->shape.end(); ++it)
	{
		it->GetPointCoords(tmp);
		pointsEig.push_back(tmp);
	}
	return pointsEig;
}

void 
Measurement::SetShape(const ::std::vector<Point>& shape)
{
	this->shape = shape;
}

::std::vector<double>
Measurement::GetArcLength() const 
{
	return this->arcLength;
}

void 
Measurement::SetArcLength(const ::std::vector<double>& s)
{
	this->arcLength = s;
}

::std::ostream& 
operator << (::std::ostream& os, Measurement& meas)
{
	::std::vector<double> conf;
	conf = meas.GetConfiguration();
	os << "Configuration:";
	for(::std::vector<double>::iterator it = conf.begin(); it != conf.end(); ++it)
		os << *it << " ";

	os << ::std::endl;

	::std::vector<Point> shape = meas.GetShape();
	::std::vector<double> arcL = meas.GetArcLength();

	int pointCounter = 1;
	double x, y, z, s;
	for(::std::vector<Point>::iterator it = shape.begin(); it != shape.end(); ++it)
	{
		it->GetPointCoords(x, y, z);
		os << "Point " << pointCounter << ": x:" << x << ", y:" << y << ", z:" << z << ", s:" << arcL[pointCounter - 1] << ::std::endl;
		pointCounter++;
	}

	//::std::cout << ::std::endl;
	return os;
}

void BuildShapeDatasetFromString(const char* filename, ShapeDataset& dataset)
{

	TiXmlDocument doc(filename);
	if (!doc.LoadFile()) return;

	TiXmlHandle hDoc(&doc);
	TiXmlElement* pElem, *pElemChild;

	TiXmlHandle hRoot(0);

	// parse all measurements
	TiXmlElement* pElemRoot ;
	pElemRoot = hDoc.FirstChildElement().Element();
	hRoot = TiXmlHandle(pElemRoot);

	pElem = hRoot.FirstChildElement("Measurement").Element();

	dataset.resize(0);

	double s, x, y, z;

	::std::string configuration;
	::std::vector<double> configurationDVector;

	Measurement tmp;
	::std::vector<Point> tmpShape;
	::std::vector<double> tmpArc;
	Point tmpPoint;

	for (pElem; pElem; pElem=pElem->NextSiblingElement())
	{
		tmpShape.clear();
		tmpArc.clear();

		pElemChild = pElem->FirstChildElement("Configuration");
		configuration = pElemChild->GetText();
		configurationDVector = DoubleVectorFromString(configuration);

		tmp.SetConfiguration(configurationDVector);

		pElemChild = pElem->FirstChildElement("Shape");
		pElemChild = pElemChild->FirstChildElement("Point");

		for (pElemChild; pElemChild; pElemChild=pElemChild->NextSiblingElement())
		{
			pElemChild->QueryDoubleAttribute("s", &s);
			tmpArc.push_back(s);

			pElemChild->QueryDoubleAttribute("x", &x);
			pElemChild->QueryDoubleAttribute("y", &y);
			pElemChild->QueryDoubleAttribute("z", &z);
			tmpPoint.SetPointCoords(x, y, z);
			tmpShape.push_back(tmpPoint);

		}
		tmp.SetShape(tmpShape);
		tmp.SetArcLength(tmpArc);
		dataset.push_back(tmp);
	}
	
}

