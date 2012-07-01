// CameraPose.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include "Homography.h"
#include "Camera.h"
#include "SiftFeature.h"
#include "BundleAdjust.h"
#include <iostream>

const double radius = 10000.0;
const int BA_ITER = 100;

//
void OptimizePair( CCamera& cam1, CCamera& cam2,
				  double* dR,
				  const CFeatureArray& set1,
				  const CFeatureArray& set2, 
				  const MatchArray& aryInlier );

//
void OptimizeSingle( const CCamera& cam1, CCamera& cam2,
					double* dR,
					const CFeatureArray& set1,
					const CFeatureArray& set2, 
					const MatchArray& aryInlier );

//
void OptimizeSet( CCamera* cam,
				  const CHomography* homo, 
				  const CFeatureArray* set,
				  const MatchArray* aryMatch,
				  int numImage );

void GetHomographyInliers( MatchArray& aryInlier,
						  const MatchArray& aryMatch,
						  const CFeatureArray& set1,
						  const CFeatureArray& set2,
						  const CHomography& homo,
						  float tol );

void PoseFromHomography( double* R, const CHomography& homo, double* K1, double* K2 );
				   

void TraceMat( const CvMat* mat )
{
	for( int i=0; i<mat->rows; ++i )
	{
		for( int j=0; j<mat->cols; ++j )
			printf( "%.4f ", mat->data.db[i*mat->cols+j] );

		printf( "\n" );
	}

	printf( "\n" );
}

int main(int argc, char* argv[])
{
	bool bOpt = false;

	// the number of images in the sequence
	int ib = 0;
	int ie = 0;
	int numImage = 0;
	
	// the intrinsic camera parameters
	double f = 0;
	double u, v, alpha;
	
	// homography inlier tolerance
	float tol = 2.f;

	int arg = 0;
	while( ++arg < argc) 
	{ 
		if( !strcmp(argv[arg], "-ib") )
			ib = atoi( argv[++arg] );

		if( !strcmp(argv[arg], "-ie") )
			ie = atoi( argv[++arg] );

		if( !strcmp(argv[arg], "-f") )
			f = atof( argv[++arg] );

		if( !strcmp(argv[arg], "-u") )
			u = atof( argv[++arg] );

		if( !strcmp(argv[arg], "-v") )
			v = atof( argv[++arg] );

		if( !strcmp(argv[arg], "-alpha") )
			alpha = atof( argv[++arg] );

		if( !strcmp(argv[arg], "-tol") )
			tol = atof( argv[++arg] );
	}

	// an initial guess of the focal length
	if( f == 0 )
		f = u*4;

	// the indices of the image sequence
	numImage = ie-ib;
	int* imgId = new int[ numImage ];
	for( int i=0; i<numImage; ++i )
		imgId[i] = ib+i;

	// the feature set for each image
	CFeatureArray* ftSet = new CFeatureArray[ numImage ];

	// the match array for each consecutive image pair
	MatchArray* aryMatch = new MatchArray[ numImage ];

	// the camera poses for each image
	CCamera* cam = new CCamera[ numImage ];

	// the homographies for each consecutive image pair
	CHomography* homo = new CHomography[ numImage ];

	// the intrinsic camera matrix
	const double dK[9] = {
		f,       0, u,
		0, f*alpha, v,
		0,       0, 1};

	for( int i=0; i<numImage; ++i )
	{
		// load the features for each image
		char strSift[128];
		sprintf( strSift, "%04d.key", imgId[i] );
		LoadSiftFromFile( ftSet[i], strSift );

		// initialize camera intrinsic parameters
		cam[i].SetIntMatrix( dK );
	}	

	// 
	for( int j=0; j<numImage; ++j )
	{
		// index of the previous camera
		int i = (j==0) ? numImage-1 : j-1;
		
		// Step 1: load the homography of image pair < imgId[i-1], imgId[i] >
		LoadHomography( homo[j], imgId[i], imgId[j] );

		// step 2: load the matches of image pair < imgId[i-1], imgId[i] >
		LoadMatchArray( aryMatch[j], imgId[i], imgId[j] );

		// step 3: estimate camera rotation from the homography 
		double dR[9];
		double dK1[9];
		double dK2[9];
		cam[i].GetIntMatrix( dK1 );
		cam[j].GetIntMatrix( dK2 );
			
		PoseFromHomography( dR, homo[j], dK1, dK2 );

		// step 4: keep homography inliers
		GetHomographyInliers( aryMatch[j], aryMatch[j], ftSet[i], ftSet[j], homo[j], tol );
			
		// step 5: perform bundle adjustment to find the best R and f
		//         If this is the first pair, we need to estimate R, f1, and f2.
		//         Otherwise we can fix cam[i-1] and estimate R and f2.
		if( j==0 )
			OptimizePair( cam[i], cam[j], dR, ftSet[i], ftSet[j], aryMatch[j] );

		else
			OptimizeSingle( cam[i], cam[j], dR, ftSet[i], ftSet[j], aryMatch[j] );
	}

	//TODO: optimize the camera poses using bundle adjustment,
	//      and store the result back to cam[i]
	OptimizeSet( cam, homo, ftSet, aryMatch, numImage );

	// Save camera poses
	for( int i=0; i<numImage; ++i )
		SaveCamera( cam[i], imgId[i] );

	delete [] homo;
	delete [] cam;
	delete [] aryMatch;
	delete [] ftSet;
	delete [] imgId;

	return 0;
}



void GetHomographyInliers( MatchArray& aryInlier,
						  const MatchArray& aryMatch,
						  const CFeatureArray& set1,
						  const CFeatureArray& set2,
						  const CHomography& homo,
						  float tol )
{
	float SQR_TOL = tol*tol;

	aryInlier.resize( aryMatch.size() );

	int k=0;
	for( int i=0; i<aryMatch.size(); ++i )
	{
		const CFeature* ft1 = set1[ aryMatch[i].first ];
		const CFeature* ft2 = set2[ aryMatch[i].second ];

		CvPoint2D64f pt = homo * cvPoint2D64f( ft1->x, ft1->y );

		double dx = pt.x -ft2->x;
		double dy = pt.y -ft2->y;

		if( dx*dx +dy*dy < SQR_TOL ) // a homography inlier
			aryInlier[k++] = aryMatch[i];	
	}

	aryInlier.resize(k);
}




/**
 *	PoseFromHomography:
 *		Input:
 *			K1 - the 3x3 intrinsic camera matrix for the first camera
 *          K2 - the 3x3 intrinsic camera matrix for the second camera
 *			homo - the homography: X2 = homo * X1
 *
 *		Ouput:
 *			R - the 3x3 rotation matrix 
 */

void PoseFromHomography( double* R, const CHomography& homo, double* K1, double* K2 )
{
    // Get the initial rotation (relative to cam1) from the homography. 
    // This can be derived from:
    //
    //     R = inv(K2) x H x K1 (ref).
    //
    // However, since K1 and K2 may not be very accurate, the rotation matrix 
    // can be of poor quality. We will use bundle adjustment to fix it later.

    //ToDo3: Compute R
    double iK2[9], temp1[9], temp2[9], temp3[9];
    for(int i = 0; i< 9; i++) {
        iK2[i] = 0;
        temp1[i] = 0;
        temp2[i] = 0;
        temp3[i] = 0;
    }
    CvMat mK1 = cvMat( 3, 3, CV_64FC1, K1);
    CvMat mK2 = cvMat( 3, 3, CV_64FC1, K2);
    CvMat mH = cvMat( 3, 3, CV_64FC1, (double *)homo.d);
    CvMat imK2 = cvMat( 3, 3, CV_64FC1, iK2);
    CvMat mtemp = cvMat( 3, 3, CV_64FC1, temp1);
    CvMat mR = cvMat( 3, 3, CV_64FC1, temp2); 
    
    cvInvert(&mK2, &imK2);
    cvMatMul(&imK2, &mH, &mtemp);
    cvMatMul(&mtemp, &mK1, &mR);

    for(int j = 0; j < 9; j++) {
        R[j] = cvmGet(&mR, j/3, j%3 );
    }
    
}

/**
 *	OptimizePair:
 *		Input:
 *			cam1 - the first camera with its intrinsic matrix and pose initialized ([R|T]=[I|0])
 *          cam2 - the second camera with its intrinsic matrix initialized
 *			dR - an initial relative 3x3 camera rotation matrix
 *          set1 - SIFT features in the first image
 *          set2 - SIFT features in the second image
 *          aryInlier - the homography iniliers
 *
 *		Ouput:
 *			cam1 - update cam1's optimized folcal length
 *          cam2 - update cam2's optimized focal length and pose
 */
void OptimizePair( CCamera& cam1, CCamera& cam2,
				  double* dR,
				  const CFeatureArray& set1,
				  const CFeatureArray& set2, 
				  const MatchArray& aryInlier )
{
	CBundleAdjust ba( 2, BA_ITER );

	// Step 1. To perform bundle adjustment, we initialize cam1 and cam2 
	//         as [K][I|0} and [K][R|0] respectively and optimize R using 
	//         bundle adjustment.
	double dRod[3];
	CvMat matRod = cvMat( 3, 1, CV_64F, dRod );
	CvMat matR = cvMat( 3, 3, CV_64F, dR );

	cvRodrigues2( &matR, &matRod );
	cam2.SetPose( dRod );

	// Set cameras
	ba.SetCamera( &cam1, 0 );
	ba.SetCamera( &cam2, 1 );

	// Step 2. We still need to create a set of 3D points. From each homography inlier, 
	//         a 3D point can be initialized by locating it on the ray that goes through 
	//         its projection.	
	for( int i=0; i<aryInlier.size(); ++i )
	{
		const CFeature* ft1 = set1[ aryInlier[i].first ];
		const CFeature* ft2 = set2[ aryInlier[i].second ];

		double dir[3];
		cam1.GetRayDirectionWS( dir, cvPoint2D64f( ft1->x, ft1->y ) );
		
		// the initialized 3d position
		CvPoint3D64f pt3 = cvPoint3D64f( dir[0]*radius, dir[1]*radius, dir[2]*radius );

		// set the 3d point and its projections in both images
		ba.SetPointProjection( pt3, 0, cvPoint2D64f( ft1->x, ft1->y ) );
		ba.SetPointProjection( pt3, 1, cvPoint2D64f( ft2->x, ft2->y ) );
	}

	// perform bundle adjustment
	ba.DoMotionAndStructure();

	// retrieve the optimized cameras
	ba.GetAdjustedCamera( &cam1, 0 );
	ba.GetAdjustedCamera( &cam2, 1 );
}

/**
 *	OptimizePair:
 *		Input:
 *			cam1 - the first camera (already optimized)
 *          cam2 - the second camera with its intrinsic matrix initialized
 *			dR - an initial relative 3x3 camera rotation matrix
 *          set1 - SIFT features in the first image
 *          set2 - SIFT features in the second image
 *          aryInlier - the homography iniliers
 *
 *		Ouput:
 *          cam2 - update cam2's optimized focal length and pose
 */
void OptimizeSingle( const CCamera& cam1, CCamera& cam2,
					double* dR,
					const CFeatureArray& set1,
					const CFeatureArray& set2, 
					const MatchArray& aryInlier )
{
	// Step 1. Initialize the camera pose of cam2

	// cam2's relative rotation to cam1
	CvMat matR = cvMat( 3, 3, CV_64F, dR );

	// cam1's absolute rotation
	double dRod1[3];
	CvMat matRod1 = cvMat( 3, 1, CV_64F, dRod1 );
	cam1.GetPose( dRod1 );

	double dRot1[9];
	CvMat matRot1 = cvMat( 3, 3, CV_64F, dRot1 );
	cvRodrigues2( &matRod1, &matRot1 );

	// compose R and Rot1 to get cam2's initial absolute rotation
	cvMatMul( &matR, &matRot1, &matR );

	double dRod2[3];
	CvMat matRod2 = cvMat( 3, 1, CV_64F, dRod2 );

	cvRodrigues2( &matR, &matRod2 );
	cam2.SetPose( dRod2 );

	// Step 2. Now we can perform bundle adjustment for cam2
	CBundleAdjust ba( 1, BA_ITER );
	ba.SetCamera( &cam2, 0 );

	// set points
	for( int i=0; i<aryInlier.size(); ++i )
	{
		const CFeature* ft1 = set1[ aryInlier[i].first ];
		const CFeature* ft2 = set2[ aryInlier[i].second ];

		double dir[3];
		cam1.GetRayDirectionWS( dir, cvPoint2D64f( ft1->x, ft1->y ) );
		
		// the 3d position
		CvPoint3D64f pt3 = cvPoint3D64f( dir[0]*radius, dir[1]*radius, dir[2]*radius );

		ba.SetPointProjection( pt3, 0, cvPoint2D64f( ft2->x, ft2->y ) );
	}

	ba.DoMotion();

	ba.GetAdjustedCamera( &cam2, 0 );
}

/**
 * Please follow similar procedure as in OptimizePair and OptimizeSingle
 * to optimize cameras altogether.
 */
void OptimizeSet( CCamera* cam,
                  const CHomography* homo, 
                  const CFeatureArray* set,
                  const MatchArray* aryMatch,
                  int numImage )
{
    //ToDo5: Extra credit

    CBundleAdjust ba( numImage, BA_ITER );

    for( int i = 0; i < numImage; i++ )
        ba.SetCamera(&cam[i], i);

    for( int j = 0; j<numImage; j++)
    {
        MatchArray aryInlier = aryMatch[j];
       
        int pre_j = (j==0) ? numImage-1 : j-1;

        CFeatureArray set1 = set[pre_j];
        CFeatureArray set2 = set[j];

        for( int k=0; k<aryInlier.size(); k++ )
        {
            const CFeature* ft1 = set1[ aryInlier[k].first ];
            const CFeature* ft2 = set2[ aryInlier[k].second ];

            double dir[3];
            cam[pre_j].GetRayDirectionWS( dir, cvPoint2D64f( ft1->x, ft1->y ) );

            CvPoint3D64f pt3 = cvPoint3D64f( dir[0]*radius, dir[1]*radius, dir[2]*radius );

            ba.SetPointProjection( pt3, j, cvPoint2D64f( ft2->x, ft2->y ) );
            ba.SetPointProjection( pt3, pre_j, cvPoint2D64f( ft1->x, ft1->y ) );
        }

    }
    ba.DoMotionAndStructure();

    for(int i=0; i < numImage ; i++)
    {
        ba.GetAdjustedCamera( &cam[i], i );
    }

}

