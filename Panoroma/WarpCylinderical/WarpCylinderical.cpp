// WarpCylinderical.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <limits.h>
#include <algorithm>
#include "Homography.h"
#include "Camera.h"

#include <cv.h>
#include <highgui.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

int main(int argc, char* argv[])
{
    // the size of the the panorama image
    CvSize szPano = cvSize( 8000, 480 );

    // the radius
    const double r = (double)szPano.width / (2*M_PI);
	
    // the pan angle per pixel
    const double angPixel =  2*M_PI / (double)szPano.width;

    // the number of images in the sequence
    int ib = 0;
    int ie = 0;
    int numImage = 0;

    int arg = 0;
    while( ++arg < argc) 
    {
        if( !strcmp(argv[arg], "-ib") )
            ib = atoi( argv[++arg] );

        if( !strcmp(argv[arg], "-ie") )
            ie = atoi( argv[++arg] );
    }	

    // the indices of the image sequence
    numImage = ie-ib;

    int* imgId = new int[ numImage ];
        
    for( int i=0; i<numImage; ++i )
        imgId[i] = ib+i;

    // the output panorama
    IplImage* imgPano = cvCreateImage( szPano, IPL_DEPTH_8U, 1 );

    IplImage* weightPano = cvCreateImage( szPano, IPL_DEPTH_32F, 1 );
    IplImage* imgFloatPano = cvCreateImage( szPano, IPL_DEPTH_32F, 1 );

    cvSetZero( imgPano );
    cvSetZero( weightPano );
    cvSetZero(imgFloatPano);
//
    cvSetZero( imgPano );

    IplImage* cv_image[numImage];

    for( int i=0; i<numImage; ++i) {
        char strName[128];
        sprintf( strName, "image%04d.pgm", imgId[i] ); // modify according to your image name
        std::cout << strName << std::endl;
        cv_image[i] = cvLoadImage( strName, 0 );
    }
        
    int count;
    double avg_int1 = 0, avg_int2 = 0;
//        for( int temp =0; temp < 20; temp++) {
    for( int i=0; i<numImage; i++) {
        count = 0;
        int _avg_int1 = 0;
        int _avg_int2 = 0;
            
        for(int j=0; j < cv_image[i]->height; j++) {
            for(int k=0; k < cv_image[i]->width; k++) {
                _avg_int1 += cvGetReal2D( cv_image[i], j, k );
                count ++;
            }
        }
        _avg_int1 /= (double)count;
        avg_int1 = 0.0 * avg_int1 + 1*_avg_int1;
        //avg_int1 /=2;

        count = 0;
        for(int j=0; j < cv_image[(i + 1)%numImage ]->height; j++) {
            for(int k=0; k < cv_image[(i+1)%numImage ]->width; k++) {
                _avg_int2 +=
                    cvGetReal2D( cv_image[(i+1)%numImage ], j, k );
                count ++;
            }
        }
        _avg_int2 /= (double)count;
        avg_int2 = 0.0 * avg_int2 + 1.0*_avg_int2;
        //avg_int2 /=2;
        for(int j=0; j < cv_image[i]->height; j++) {
            for(int k=0; k < cv_image[i]->width; k++) {
                double diff = avg_int2 - avg_int1;
//                    double diff = 128 - avg_int1;
                double val = cvGetReal2D( cv_image[i], j, k );
                val += diff/2.0; 
                cvSetReal2D( cv_image[i], j, k, val );
            }
        }
            
        for(int j=0; j < cv_image[(i + 1)%numImage ]->height; j++) {
            for(int k=0; k < cv_image[(i+1)%numImage ]->width; k++) {
                double diff = avg_int1 - avg_int2;
//                    double diff = 128 - avg_int2;
                double val = cvGetReal2D( cv_image[(i+1)%numImage ], j, k );
                val += diff/2.0; 
                cvSetReal2D( cv_image[(i+1)%numImage ], j, k, val );
            }
        }
            
    }

    // Create weight template
    double maxWt=0;

//    IplImage* weightTemplate = cvCreateImage( cvSize(cv_image[0]->width,cv_image[0]->height),
//                                              IPL_DEPTH_32F, 1 );
//    cvSetZero( weightTemplate );
    double **weight;
    
    weight = (double **)malloc(sizeof(double*) * cv_image[0]->height);
    for(int i=0; i<cv_image[0]->height; i++) {
        weight[i] = (double *)calloc(cv_image[0]->width, sizeof(double));
    }
    
//    std::vector< double > weight_temp(cv_image[0]->width);
//    std::vector< weight_temp > weight(cv_image[0]->height);
    for(int i = 0; i < cv_image[0]->height; i++)
        for(int j = 0; j < cv_image[0]->width; j++)
            weight[i][j] = 0.0;
    int cr = cv_image[0]->height/2;
    int cc = cv_image[0]->width/2;
    // Initialize weight template
    for(int i = 0; i < cv_image[0]->height; i++)
    {
        for(int j = 0; j < cv_image[0]->width; j++)
        {
            double value = (1.0 - (abs(cr - i)/((float)cr)) + 1.0 - (abs(cc - j)/((float)cc)))/2;
            if(value > maxWt)
                maxWt = value;
//            cvSetReal2D(weightTemplate, i, j, value);
            weight[i][j] = value;
        }
    }
    //cvSaveImage( "wtemplate.jpg", weightTemplate );
    printf(" \n Max weight:%lf",maxWt);


                    
    for( int i=0; i<numImage; ++i ) // for each image
    {
        // load the iamge
        // char strName[128];
        // sprintf( strName, "image1%d.pgm", imgId[i] ); // modify according to your image name
        // IplImage* img = cvLoadImage( strName, 0 );
        IplImage* img = cv_image[i];
        // load the camera
        CCamera cam;
        LoadCamera( cam, imgId[i] );

        //
        double dir[3];
		
        CvPoint2D64f corner[4] = { 
            cvPoint2D64f( 0, 0 ),
            cvPoint2D64f( 0, img->height ),
            cvPoint2D64f( img->width, 0 ),
            cvPoint2D64f( img->width, img->height ) };
		
        int lm = INT_MAX; // left most
        int rm = -INT_MAX; // right most

        // find the left end of the image in the panorama
        for( int j=0; j<2; ++j )
        {
            cam.GetRayDirectionWS( dir, corner[j] );			
            double theta = atan2( dir[0], dir[2] );
            lm = ( lm < floor( szPano.width *theta /(2*M_PI) ) ) ? lm : floor( szPano.width *theta /(2*M_PI) );
        }

        // find the right end of the image in the panorama
        for( int j=0; j<2; ++j )
        {
            cam.GetRayDirectionWS( dir, corner[j+2] );			
            double theta = atan2( dir[0], dir[2] );
            rm = ( rm > ceil( szPano.width *theta /(2*M_PI) ) ) ? rm : ceil( szPano.width *theta /(2*M_PI) );
        }

        if( lm > rm ) // 180 degree crossing
            lm -= szPano.width;

        printf( "L:%d, R:%d\n", lm, rm );

        // ToDo 4: Image blending
        for( int m=0; m<szPano.height; ++m )
            for( int n=lm; n<rm; ++n )
            {
                double theta = n * angPixel;
                double tan_phi = ( m-szPano.height/2 )/r;

                // 
                CvPoint3D64f pt = cvPoint3D64f( r*sin( theta ), r*tan_phi, r*cos( theta ) );
			
                // check if the point is in front of the camera
                CvPoint2D64f ptProj = cam.GetProjection( pt );
				
                int bx = ptProj.x +0.5;
                int by = ptProj.y +0.5;

                if( bx >=0 && bx < img->width && by >=0 && by < img->height )
                {
                    double v = cvGetReal2D( img, by, bx );
                    //double wt = cvGetReal2D( weightTemplate, by, bx )/1.05;
                    double wt = weight[by][bx];
                    cvSetReal2D( imgPano, m, n<0? n+szPano.width: n, v );
                    double temp =cvGetReal2D( imgFloatPano, m, n<0? n+szPano.width: n);
                    temp += (v/255)*wt;
                    cvSetReal2D( imgFloatPano, m, n<0? n+szPano.width: n, temp );
//for linearimgFloatPano

                    // Used for linear method.
                    double pano_wt =cvGetReal2D( weightPano, m, n<0? n+szPano.width: n);
                    cvSetReal2D( weightPano, m, n<0? n+szPano.width: n, pano_wt+wt );

                }
            }
    }

    IplImage* wresult = cvCreateImage(szPano,IPL_DEPTH_32F,1);
    cvDiv(imgFloatPano,weightPano,wresult);

    cvSaveImage( "panorama.jpg", wresult );
       
//    cvSaveImage( "panorama.jpg", imgPano );
    cvReleaseImage( &imgPano );

    return 0;
}

