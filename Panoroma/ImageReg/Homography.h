#ifndef HOMO
#define HOMO

#include <cv.h>

class CHomography: 
	public CvMat 
{
public:

	// the elements for a 3x3 homography matrix
	double d[9];  
	
	CHomography(void);
	
	virtual ~CHomography(void);

	// matrix multiplication
	CHomography operator*( const CHomography& homo ) const;

	CHomography& operator=( const CHomography& homo );

	// Perform point transformation through the homography
	CvPoint2D64f operator*( const CvPoint2D64f& pt ) const;
};

// Save the homoraphy from image i to image j
void SaveHomography( const CHomography& homo, int i, int j );

// Load the homoraphy from image i to image j
void LoadHomography( CHomography& homo, int i, int j );

#endif

