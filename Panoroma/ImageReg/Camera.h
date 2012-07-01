#pragma once

#include <cv.h>

class CCamera
{
private:

	// Intrinsic parameters
	//     [ f,       0, u ]
	// K = [ 0, alpha*f, v ]
	//     [ 0,       0, 1 ]
	double f, u, v, alpha;

	// Extrinsic params. Use Rodrigues representation for the rotation.
	//
	// M = [ R_3x3 | t_3x1 ]
	//
	double rod[3], t[3];

	// the projection matrix K*M
	double d[12];

	void UpdateProjMat();

public:

	CCamera(void);

	CCamera( double f, double u=0, double v=0, double alpha=1, const double* rod=NULL, const double* t=NULL );

	// copy constructor
	CCamera( const CCamera& cam );

	~CCamera(void);

	CCamera& operator=( const CCamera& cam );

	// set/get the 3x3 intrinsic camera matrix
	void SetIntMatrix( const double* mat );
	void GetIntMatrix( double* mat ) const;

	// set/get the 3x4 extrinsic camera matrix
	void SetExtMatrix( const double* mat );
	void GetExtMatrix( double* mat ) const;

	//
	void SetFocalLength( double f );
	double GetFocalLength() const;

	//
	void SetAlpha( double alpha );
	double GetAlpha() const;

	//
	void SetImageCenter( const CvPoint2D64f& pt );
	CvPoint2D64f GetImageCenter() const;

	//
	void SetPose( const double* rod = NULL, const double* t = NULL );
	void GetPose( double* rod = NULL, double* t = NULL ) const;		

	// the projection of a 3D point in the world coordinate
	CvPoint2D64f GetProjection( const CvPoint3D64f& pt ) const;

	// given a 3d point in camera space, return the 3d point position in the world space
	CvPoint3D64f GetPositionWS( CvPoint3D64f ptcs ) const;

	// given a 2d point in the image, get the ray direction in the world coordinate
	void GetRayDirectionWS( double dir[3], const CvPoint2D64f& pt ) const;

};


void LoadCamera( CCamera& cam, int i );
void SaveCamera( const CCamera& cam, int i );

