#ifndef BUND_ADJ
#define BUND_ADJ

#include "Camera.h"

#include <map>
#include <functional>
using namespace std;

struct Projections
{
	int numProj;
	CvPoint3D64f ptAdj;
	CvPoint2D64f* pos;
	bool* vmask;

	Projections()
	{
		numProj = 0;
		ptAdj = cvPoint3D64f( 0, 0, 0 );
		pos = NULL;
		vmask = NULL;
	}

	Projections( int cam_count )
	{
		numProj = cam_count;
		ptAdj = cvPoint3D64f( 0, 0, 0 );
		
		pos = new CvPoint2D64f[ cam_count ];
		
		vmask = new bool[ cam_count ];
		memset( vmask, 0, sizeof(bool)*cam_count );
	}

	Projections( const Projections& proj )
	{
		numProj = proj.numProj;
		ptAdj = proj.ptAdj;

		pos = new CvPoint2D64f[ numProj ];
		memcpy( pos, proj.pos, sizeof(CvPoint2D64f)*numProj );

		vmask = new bool[ numProj ];
		memcpy( vmask, proj.vmask, sizeof(bool)*numProj );
	}

	Projections& operator=( const Projections& proj )
	{
		if( numProj>0 )
		{
			delete [] pos;
			delete [] vmask;
		}

		numProj = proj.numProj;

		ptAdj = proj.ptAdj;

		pos = new CvPoint2D64f[ numProj ];
		memcpy( pos, proj.pos, sizeof(CvPoint2D64f)*numProj );

		vmask = new bool[ numProj ];
		memcpy( vmask, proj.vmask, sizeof(bool)*numProj );

		return *this;
	}

	~Projections()
	{
		if( pos )
		{
			delete [] pos;
			pos = NULL;
		}

		if( vmask )
		{
			delete [] vmask;
			vmask = NULL;
		}
	}
};

const double eps = 0.00001;
//code changed by me
class cvPointLess
    : public std::binary_function<const CvPoint3D64f, const CvPoint3D64f, bool>
{
public:
bool operator ()(const CvPoint3D64f& Left, const CvPoint3D64f& Right) const
{

	if( Left.z < Right.z-eps )
		return true;

	if( Left.z > Right.z+eps )
		return false;

	if( Left.y < Right.y-eps )
		return true;

	if( Left.y > Right.y+eps )
		return false;

	if( Left.x < Right.x-eps )
		return true;

	if( Left.x > Right.x+eps )
		return false;

	return false;
}
};
//end of code change
typedef map< CvPoint3D64f, Projections, cvPointLess >::iterator MapProjIter;

class CBundleAdjust
{
protected:

	int m_maxIter; // number of interations

	int m_numCamera;

	CCamera* m_cam;

	map< CvPoint3D64f, Projections, cvPointLess > m_PointProjection;

protected:

	CBundleAdjust();

	const char* sba_term_string( double code );

	void FormatInputFile();

public:

	CBundleAdjust( int numView, int maxIter = 1000 );

	~CBundleAdjust();

	//
	int GetMaxIteration();
	void SetMaxIteration( int iter );

	//
	void SetPointProjection( const CvPoint3D64f& pt, int idxCam, const CvPoint2D64f& ptProj );
	void RemovePoint( const CvPoint3D64f& pt );

	// Get the adjusted point pt 
	void GetAdjustedPoint( CvPoint3D64f& ptAdj, const CvPoint3D64f& pt );

	// Set the camera to be adjusted
	void SetCamera( const CCamera* cam, int idxCam );

	// Get the adjusted camera 
	void GetAdjustedCamera( CCamera* cam, int idxCam );

	// optimize camera paramters and 3d point locations jointly
	void DoMotionAndStructure();

	// optimize only the camera paramters
	void DoMotion(); 

	// optimize only the 3d point locations
	//virtual void DoStructure(){} 

	
};

#endif

