#ifndef SIFT
#define SIFT

#include "Feature.h"

class CSiftFeature :
	public CFeature
{
public:

	float m_scale;
	
	float m_angle;
	
	CSiftFeature( int numDim );
	
	virtual ~CSiftFeature(void);
};

// Load SIFT features from a file
void LoadSiftFromFile( CFeatureArray& fs, const char* strFileName );

// Match SIFT features between two feature sets
void MatchSiftFeatures( MatchArray& aryMatch, const CFeatureArray& set1, const CFeatureArray& set2 ); 

#endif

