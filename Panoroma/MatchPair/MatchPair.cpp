// MatchPair.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include "SiftFeature.h"

int main(int argc, char* argv[])
{
	int i = -1;
	int j = -1;

	int arg = 0;
	while( ++arg < argc) 
	{ 
		if( !strcmp(argv[arg], "-i") )
			i = atoi( argv[++arg] );

		if( !strcmp(argv[arg], "-j") )
			j = atoi( argv[++arg] );
	}	

	try
	{
		CFeatureArray set_i, set_j;

		// Step 1: load the extracted features using LoadSiftFromFile()
		char strBuf[128];
		sprintf( strBuf, "%04d.key", i );
		LoadSiftFromFile( set_i, strBuf );
		
		sprintf( strBuf, "%04d.key", j );
		LoadSiftFromFile( set_j, strBuf );

		// Step 2: match features
		MatchArray aryMatch;
		MatchSiftFeatures( aryMatch, set_i, set_j );

		// Step 3: Save the matches
		SaveMatchArray( aryMatch, i, j );
	}
	catch( exception& err )
	{
		printf( "%s\n", err.what() );
	}

	return 0;
}

