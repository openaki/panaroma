// AlignPair.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include "SiftFeature.h"
#include "Homography.h"
#include <time.h>
#include <algorithm>
#include <stdlib.h>
#include <sys/time.h>
#include <iostream>
#include <math.h>
using namespace std;

void RansacHomography( CHomography& homo, 
					  const MatchArray& aryMatch, 
					  const CFeatureArray& set1, 
					  const CFeatureArray& set2, 
					  float inlierTol, int numIter );

int main(int argc, char* argv[])
{
	int i = -1;
	int j = -1;
	float inlierTol = 1;
	int numIter = 1000;

	// ransac
	srand( time(NULL) );

	int arg = 0;
	while( ++arg < argc) 
	{ 
		if( !strcmp(argv[arg], "-i") )
			i = atoi( argv[++arg] );

		if( !strcmp(argv[arg], "-j") )
			j = atoi( argv[++arg] );

		if( !strcmp(argv[arg], "-tol") )
			inlierTol = atof( argv[++arg] );

		if( !strcmp(argv[arg], "-iter") )
			numIter = atoi( argv[++arg] );
	}	

	try
	{
		char strBuf[128];

		// Step 1: load the extracted features
		CFeatureArray set_i, set_j;
		
		sprintf( strBuf, "%04d.key", i );
		LoadSiftFromFile( set_i, strBuf );
		
		sprintf( strBuf, "%04d.key", j );
		LoadSiftFromFile( set_j, strBuf );

		// Step 2: load the matches
		MatchArray aryMatch;
		LoadMatchArray( aryMatch, i, j );

		// Step 3: Estimate the homography
		CHomography homo;
		RansacHomography( homo, aryMatch, set_i, set_j, inlierTol, numIter );

		// Step 4: Save the homography
		SaveHomography( homo, i, j );
	}
	catch( exception& err )
	{
		printf( "%s\n", err.what() );
	}

	return 0;
}

/**
 *	RansacHomography:
 *		Input:
 *			aryMatch - an array of potential matches between two images
 *			inlierTol - the tolerance to regard a match as an inlier
 *			numIter - number of iterations for Ransac
 *
 *		Ouput:
 *			homo - the best estimated homography (with the max nubmer of inliers)
 */
void RansacHomography( CHomography& homo, 
                       const MatchArray& aryMatch, 
                       const CFeatureArray& set1, 
                       const CFeatureArray& set2, 
                       float inlierTol, int numIter )
{
	const float SQR_TOL = inlierTol*inlierTol;
	const int NUM_SAMP = 6;
	int maxInlier = 0;
	double dA[ NUM_SAMP*2*8 ];
	double dB[ NUM_SAMP*2 ];
        double dH[8];
        struct timeval tv;
        gettimeofday(&tv, NULL);
        srand48(tv.tv_usec);
        long int index;
        int flag;
        vector < int > dup_index;
        int test_index[] = {100, 150, 200, 250, 300, 350};
// ToDo2: Find homography using RANSAC 

        for(int i=0; i < NUM_SAMP; i++) {
            dB[i] = 0;
        }

        for(int i=0; i< numIter; i++) {
            dup_index.clear();
            for(int j =0 ; j< NUM_SAMP; j++ ) {
                index = lrand48();
                index = index % aryMatch.size();
//                 index = test_index[j];
                flag = 1;
                for(unsigned int k=0; k < dup_index.size(); k++) {
                    if(dup_index[k] == index) {
                        flag = 0;
                    }
                }
                if(!flag) {
                    --j;
                    continue;
                }
//                std::cout  << "Index " << index << std::endl;
                int ind1 = aryMatch[index].first;
                int ind2 = aryMatch[index].second;
                
                dA[16*j + 0] = -1 * set1[ind1]->x;
                dA[16*j + 1] = -1 * set1[ind1]->y;
                dA[16*j + 2] = -1;
                dA[16*j + 3] = 0;
                dA[16*j + 4] = 0;
                dA[16*j + 5] = 0;
                dA[16*j + 6] = set2[ind2]->x * set1[ind1]->x;
                dA[16*j + 7] = set2[ind2]->x * set1[ind1]->y;

                dA[16*j + 8 + 0] = 0;
                dA[16*j + 8 + 1] = 0;
                dA[16*j + 8 + 2] = 0;
                dA[16*j + 8 + 3] = -1 * set1[ind1]->x;
                dA[16*j + 8 + 4] = -1 * set1[ind1]->y;
                dA[16*j + 8 + 5] = -1;
                dA[16*j + 8 + 6] = set2[ind2]->y * set1[ind1]->x;
                dA[16*j + 8 + 7] = set2[ind2]->y * set1[ind1]->y;
                dB[2*j] = -1*set2[ind2]->x;
                dB[2*j + 1] = -1*set2[ind2]->y;
                
            }
            
            for(int j = 0; j < 8; j++)
                dH[j] = 0;
            CvMat mA = cvMat( NUM_SAMP * 2, 8, CV_64FC1, dA);
            CvMat mB = cvMat( NUM_SAMP * 2, 1, CV_64FC1, dB);
            CvMat mH = cvMat( 8, 1, CV_64FC1, dH);
            int ret = cvSolve(&mA, &mB, &mH, CV_SVD);
//            std::cout << ret << std::endl;
            for(int j = 0; j < 8; j++) {
                dH[j] = cvmGet(&mH, j, 0);
//                std::cout << "H is " << std::endl;
//                std::cout << dH[j] << std::endl;
            }
            //continue;
            int inlier_count = 0;
            for(unsigned int j = 0; j< aryMatch.size(); j++) {
                int ind1 = aryMatch[j].first;
                int ind2 = aryMatch[j].second;
                double x2 = set2[ind2]->x;
                double y2 = set2[ind2]->y;
                double x1 = set1[ind1]->x;
                double y1 = set1[ind1]->y;
                double _x1p = dH[0] * x1 + dH[1] * y1 + dH[2]*1;
                double _y1p = dH[3] * x1 + dH[4] * y1 + dH[5]*1;
                double h1p = dH[6] * x1 + dH[7] * y1 + 1;
                double x1p = _x1p / h1p;
                double y1p = _y1p / h1p;
                double dist = (x1p - x2)*(x1p - x2) + (y1p - y2)*(y1p - y2);
                //               std::cout << "Distance is " << sqrt(dist) << std::endl;
                if( sqrt(dist) < SQR_TOL) {
                    //  std::cout << "Coming here" << std::endl;
                    inlier_count++;
                }
            }

            if(inlier_count > maxInlier) {
                maxInlier = inlier_count;
                for(int k = 0; k < 8; k++)
                    homo.d[k] = dH[k];

                homo.d[8] = 1;
            }
        }

        
	printf( "homography inliers: %d(%d)\n", maxInlier, aryMatch.size() );
}
