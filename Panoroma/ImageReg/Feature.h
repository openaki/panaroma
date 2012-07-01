#ifndef FEATURE
#define FEATURE

#include <set>
#include <vector>

using namespace std;

// feature base class
class CFeature
{
public:

	// the location of the feature
	double x, y;

	// number of dimensions of a feature vector
	int m_numDim;

	// the data for each dimension
	float* d;	
	
	CFeature(void);
	
	virtual ~CFeature(void);
};

// feature container class
typedef vector< const CFeature* > CFeatureArray;

// A match of a feature in one image and a feature in another
typedef pair< int, int > Match;

// An array of matched features
typedef vector< Match > MatchArray;

// Load the feature indices of the matches in the image pair <i,j>
void LoadMatchArray( MatchArray& aryMatch, int i, int j );

// Save the feature indices of the matches in the image pair <i,j>
void SaveMatchArray( const MatchArray& aryMatch, int i, int j );

#endif

