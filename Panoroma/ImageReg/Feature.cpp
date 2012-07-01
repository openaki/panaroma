#include "Feature.h"

#include <fstream>
#include <string>
using namespace std;

//using namespace std;

CFeature::CFeature(void)
{
	m_numDim = 0;
	d = NULL;
}

CFeature::~CFeature(void)
{
}

// Load the feature indices of the matches in the image pair <i,j>
// ( You don't have to modify this part )
void LoadMatchArray( MatchArray& aryMatch, int i, int j )
{
	char strName[128];
	sprintf( strName, "%04d_%04d.match", i, j );

	ifstream in( strName );
	if( !in.is_open() ) 
		throw std::string( "File not found in LoadMatchArray()" );

	int numMatch;
	in >> numMatch;

	aryMatch.resize( numMatch );

	for( int i=0; i<numMatch; ++i )
		in >> aryMatch[i].first >> aryMatch[i].second;

	in.close();
}

// Save the indices of the matches in the image pair <i,j>
// ( You don't have to modify this part )
void SaveMatchArray( const MatchArray& aryMatch, int i, int j )
{
	char strName[128];
	sprintf( strName, "%04d_%04d.match", i, j );

	ofstream out( strName );

	// output the number of matches
	out << aryMatch.size() << endl;

	// output each pair of indices
	for( int i=0; i<(int)aryMatch.size(); ++i )
		out << aryMatch[i].first << " " << aryMatch[i].second << endl;

	out.close();
}

