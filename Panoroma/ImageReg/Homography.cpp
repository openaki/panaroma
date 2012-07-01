#include "Homography.h"

#include <fstream>
#include <string>
using namespace std;

CHomography::CHomography(void)
{
	// initialize as an identity matrix
	d[0] = d[4] = d[8] = 1;
	d[1] = d[2] = d[3] = d[5] = d[6] = d[7] = 0;

	cvInitMatHeader( this, 3, 3, CV_64F, d );
}

CHomography::~CHomography(void)
{
}


// matrix multiplication
CHomography CHomography::operator*( const CHomography& homo ) const
{
	CHomography hm;
	cvMatMul( (CvMat*)(this), (CvMat*)(&homo), (CvMat*)&hm );

	return hm;
}

// 
CHomography& CHomography::operator=( const CHomography& homo )
{
	memcpy( this->d, homo.d, sizeof(double)*9 );

	return *this;
}

// Perform point transformation through the homography
CvPoint2D64f CHomography::operator*( const CvPoint2D64f& pt ) const
{
	double x = d[0]*pt.x +d[1]*pt.y +d[2];
	double y = d[3]*pt.x +d[4]*pt.y +d[5];
	double z = d[6]*pt.x +d[7]*pt.y +d[8];

	return cvPoint2D64f( x/z, y/z );
}


// Save the homoraphy from image i to image j
void SaveHomography( const CHomography& homo, int i, int j )
{
	char strName[128];
	sprintf( strName, "%02d_%02d.homo", i, j );

	ofstream out( strName );

	for( int i=0; i<3; ++i )
		for( int j=0; j<3; ++j )
		{
			out << homo.d[i*3+j];

			if( j<2 )
				out << " ";
			else
				out << "\n";
		}

	out.close();
}

// Load the homoraphy from image i to image j
void LoadHomography( CHomography& homo, int i, int j )
{
	char strName[128];
	sprintf( strName, "%02d_%02d.homo", i, j );

	ifstream in( strName );
	if( !in.is_open() ) 
		throw std::string("File not found in LoadHomography()" );

	for( int i=0; i<9; ++i )
		in >> homo.d[i];

	in.close();
}


