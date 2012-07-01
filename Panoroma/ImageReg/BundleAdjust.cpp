#include "BundleAdjust.h"
#include "sba.h"
#include <fstream>
#include <string>
using namespace std;

//string exepath = "D:\\Research\\LibraryProj\\Stabilization\\release";
//const char* strMotion = "motion.txt";
//const char* strStruct = "struct.txt";
//const char* strInitMotion = "pose.txt";
const char* strPointProj = "points.txt";
//const char* strCalib = "calib.txt";


CBundleAdjust::CBundleAdjust():
	m_cam( NULL )
{
}

CBundleAdjust::CBundleAdjust( int numView, int maxIter )
{
	m_maxIter = maxIter;
	m_numCamera = numView;	

	m_cam = new CCamera[ numView ];
}

CBundleAdjust::~CBundleAdjust()
{
	delete [] m_cam;
}

//void CBundleAdjust::Clear()
//{
//	m_PointProjection.clear();
//}


const char *CBundleAdjust::sba_term_string( double code ) 
{
	if(fabsf(code - 1) < 0.00000001) {
		return "stopped by small gradient J^T e.";
	}
	else if(fabsf(code - 2) < 0.00000001) {
		return "stopped by small dp.";
	}
	else if(fabsf(code - 3) < 0.00000001) {
		return "stopped by itmax.";
	}
	else if(fabsf(code - 4) < 0.00000001) {
		return "stopped by small relative reduction in ||e||_2.";
	}
	else if(fabsf(code - 5) < 0.00000001) {
		return "too many attempts to increase damping. Restart with increased mu.";
	}
	else if(fabsf(code - 6) < 0.00000001) {
		return "stopped by small ||e||_2.";
	}
	else if(fabsf(code - 7) < 0.00000001) {
		return "stopped by invalid (i.e. NaN or Inf) \"func\" values. This is a user error.";
	}
	else
		return "Unknown";
}

void CBundleAdjust::FormatInputFile()
{
	ofstream fProj( strPointProj );

	//int i = 0;
	for( MapProjIter iter = m_PointProjection.begin(); iter != m_PointProjection.end(); iter++ )
	{
		const Projections& proj = iter->second;

		int fn = 0;
		for( int i=0; i<m_numCamera; ++i )
			fn += proj.vmask[i] ? 1 : 0;

		fProj << iter->first.x << "\t" << iter->first.y << "\t" << iter->first.z << "\t" << fn << " ";

		for( int i=0; i<m_numCamera; ++i )
		{
			if( proj.vmask[i] )
			{
				CvPoint2D64f& pt = proj.pos[i];
				fProj << i << "\t" << pt.x << "\t" << pt.y << "\t";
			}
		}
		fProj << endl;
	}

	fProj.close();
}


int CBundleAdjust::GetMaxIteration()
{
	return m_maxIter;
}


void CBundleAdjust::SetMaxIteration( int iter )
{
	m_maxIter = iter;
}


void CBundleAdjust::SetPointProjection( const CvPoint3D64f& pt, int idxCam, const CvPoint2D64f& ptProj )
{
	map< CvPoint3D64f, Projections, cvPointLess >::iterator iter = m_PointProjection.find( pt );

	// if the 3D point is new
	if( iter == m_PointProjection.end() )
	{
		pair< MapProjIter, bool > res =
			m_PointProjection.insert( pair< CvPoint3D64f, Projections >( pt, Projections( m_numCamera ) ) );

		assert( res.second );

		MapProjIter iterNew = res.first;
		iterNew->second.pos[ idxCam ] = ptProj;
		iterNew->second.vmask[ idxCam ] = true;
	}

	else
	{
		Projections& proj = iter->second;

		proj.pos[ idxCam ] = ptProj;
		proj.vmask[ idxCam ] = true;
	}
}


void CBundleAdjust::RemovePoint( const CvPoint3D64f& pt )
{
	m_PointProjection.erase( pt );
}


void CBundleAdjust::GetAdjustedPoint( CvPoint3D64f& ptAdj, const CvPoint3D64f& pt )
{
	MapProjIter iter = m_PointProjection.find( pt );

	if( iter == m_PointProjection.end() )
		printf( "point not found!" );	

	else
		ptAdj = iter->second.ptAdj;
}


void CBundleAdjust::SetCamera( const CCamera* cam, int idxCam )
{
	m_cam[idxCam] = (*cam);
}


void CBundleAdjust::GetAdjustedCamera( CCamera* cam, int idxCam )
{
	*cam = m_cam[ idxCam ];
}

struct AugData
{
	double u, v, alpha;
	double t[3];

	double* X;
};


/* Given the parameter vectors aj and bi of camera j and point i, computes in xij the
 * predicted projection of point i on image j
 */
void sba_projectRFS( int j, int i, double *aj, double *bi, double *xij, void *adata )
{
	// the fixed camera parameters
	double& u = ((AugData*)adata)->u;
	double& v = ((AugData*)adata)->v;
	double& alpha = ((AugData*)adata)->alpha;
	double* t = ((AugData*)adata)->t;

	// Camera j
	CCamera cam( *aj, u, v, alpha, aj+1, t );

	// 3D Point i
	CvPoint3D64f pt3d = cvPoint3D64f( bi[0], bi[1], bi[2] );
	
	// 2D projection
	CvPoint2D64f pt2d = cam.GetProjection( pt3d );

	xij[0] = pt2d.x;
	xij[1] = pt2d.y;
}

void CBundleAdjust::DoMotionAndStructure()
{
	FormatInputFile();

	int numPt = m_PointProjection.size();	
	printf( "3D ptCount= %d\n", numPt );

	const int camParams = 4; // 1 for focal length, and 3 for rotation 
	const int ptParams = 3; // 
	
	// parameters to be optimized
	double *params = new double[ m_numCamera*camParams + numPt*ptParams ];

	// appearance mask
	char *vmask = new char[ numPt *m_numCamera ];
	memset( vmask, 0, numPt *m_numCamera *sizeof(char) );

	// projections
	int xParams = 2;
	double *x = new double[ numPt *m_numCamera *xParams ];

	// u, v, alpha, and t are fixed
	AugData adata; 
	adata.alpha = m_cam[0].GetAlpha();
	adata.u = m_cam[0].GetImageCenter().x;
	adata.v = m_cam[0].GetImageCenter().y;
	m_cam[0].GetPose( NULL, adata.t );

	// fill in camera params
	for( int i=0; i<m_numCamera; ++i ) 
	{
		double *pCam = params + i*camParams;

		// focal length
		*pCam = m_cam[i].GetFocalLength();

		// rotation
		m_cam[i].GetPose( pCam+1, NULL );
	}	

	// fill in points and projections
	int n = 0;
	int p = 0;
	for( MapProjIter iter = m_PointProjection.begin(); iter != m_PointProjection.end(); iter++ )
	{
		const CvPoint3D64f& pt3d = iter->first;
		Projections& proj = iter->second;

		// init 3d point
		double *pPt = params + m_numCamera*camParams + n*ptParams;
		pPt[0] = pt3d.x;
		pPt[1] = pt3d.y;
		pPt[2] = pt3d.z;

		for( int j=0; j<m_numCamera; ++j )
		{
			if( proj.vmask[j] )
			{
				// vmask
				vmask[ n*m_numCamera +j ] = 1;

				// projections
				x[p++] = proj.pos[j].x;
				x[p++] = proj.pos[j].y;
			}
		}
		
		n++;
	}	
	
	double opts[SBA_OPTSSZ];
	opts[0]=SBA_INIT_MU;
	opts[1]=SBA_STOP_THRESH;
	opts[2]=SBA_STOP_THRESH;
	opts[3]=SBA_STOP_THRESH;
	//opts[3]=0.05*numprojs; // uncomment to force termination if the average reprojection error drops below 0.05
	opts[4]=0.0;
	
	double info[SBA_INFOSZ];

	//
	int iters = sba_motstr_levmar( numPt, m_numCamera, 0, vmask, params, camParams, 
	                              ptParams, x, NULL, xParams, sba_projectRFS, 
	                              NULL, &adata, m_maxIter, 1, opts, info);
	
	if( iters == SBA_ERROR ) 
		printf( "CBundleAdjust::DoMotionAndStructure() error\n" );

	else 
	{
		printf( "Bundle adjustment took %d iterations,\n", iters );
		printf( "Error at termination (||e||_2): %f\n", info[1] );
		printf( "Reason for terminating: %s\n\n", sba_term_string(info[6]) );
		//more info ...
		
		// get adjusted points
		double *pPt = params + m_numCamera*camParams;

		for( MapProjIter iter = m_PointProjection.begin(); iter != m_PointProjection.end(); iter++ )
		{
			Projections& proj = iter->second;
			proj.ptAdj = cvPoint3D64f( pPt[0], pPt[1], pPt[2] );

			pPt += 3;
		}

		// get adjusted cameras
		for( int i=0; i<m_numCamera; ++i )
		{
			double* pCamParam = params +i*camParams;

			m_cam[i].SetFocalLength( *pCamParam );
			m_cam[i].SetPose( pCamParam+1 );
		}
	}
	
	delete [] params;
	delete [] vmask;
	delete [] x;
}


/* Given the parameter vector aj of camera j, computes in xij the
 * predicted projection of point i on image j
 */
void sba_projectRF( int j, int i, double *aj, double *xij, void *adata )
{
	// the fixed camera parameters
	double& u = ((AugData*)adata)->u;
	double& v = ((AugData*)adata)->v;
	double& alpha = ((AugData*)adata)->alpha;
	double* t = ((AugData*)adata)->t;
	double* X = ((AugData*)adata)->X +3*i;

	// Camera j
	CCamera cam( *aj, u, v, alpha, aj+1, t );

	// 3D Point i
	CvPoint3D64f pt3d = cvPoint3D64f( X[0], X[1], X[2] );
	
	// 2D projection
	CvPoint2D64f pt2d = cam.GetProjection( pt3d );

	xij[0] = pt2d.x;
	xij[1] = pt2d.y;

	//odprintf( "%f %f", pt2d.x, pt2d.y );
}

void CBundleAdjust::DoMotion()
{
	FormatInputFile();

	int numPt = m_PointProjection.size();	
	printf( "3D ptCount= %d\n", numPt );

	const int camParams = 4; // 1 for focal length, and 3 for rotation
	const int ptParams = 3;
	
	double *params = new double[ m_numCamera*camParams ];

	char *vmask = new char[ numPt *m_numCamera ];
	memset( vmask, 0, numPt *m_numCamera *sizeof(char) );

	int xParams = 2;
	double *x = new double[ numPt *m_numCamera *xParams ];

	AugData adata;
	adata.alpha = m_cam[0].GetAlpha();
	adata.u = m_cam[0].GetImageCenter().x;
	adata.v = m_cam[0].GetImageCenter().y;
	m_cam[0].GetPose( NULL, adata.t );
	adata.X = new double[ numPt*ptParams ];

	//init camera params
	for( int i=0; i<m_numCamera; ++i ) 
	{
		//
		double *pCam = params + i*camParams;

		// focal length
		pCam[0] = m_cam[i].GetFocalLength();

		// R
		m_cam[i].GetPose( pCam+1 );
	}	

	int n = 0;
	int p = 0;
	for( MapProjIter iter = m_PointProjection.begin(); iter != m_PointProjection.end(); iter++ )
	{
		const CvPoint3D64f& pt3d = iter->first;
		Projections& proj = iter->second;

		// init 3d point
		double *pPt = adata.X +n*ptParams;
		pPt[0] = pt3d.x;
		pPt[1] = pt3d.y;
		pPt[2] = pt3d.z;

		for( int j=0; j<m_numCamera; ++j )
		{
			if( proj.vmask[j] )
			{
				// vmask
				vmask[ n*m_numCamera +j ] = 1;

				// projections
				x[p++] = proj.pos[j].x;
				x[p++] = proj.pos[j].y;
			}
		}
		
		n++;
	}	
	
	//normalizeCams();
	
	double opts[SBA_OPTSSZ];
	opts[0]=SBA_INIT_MU;
	opts[1]=SBA_STOP_THRESH;
	opts[2]=SBA_STOP_THRESH;
	opts[3]=SBA_STOP_THRESH;
	//opts[3]=0.05*numprojs; // uncomment to force termination if the average reprojection error drops below 0.05
	opts[4]=0.0;
	
	double info[SBA_INFOSZ];

	//
	int iters = sba_mot_levmar( numPt, m_numCamera, 0, vmask, params, camParams, 
	                              x, NULL, xParams, sba_projectRF, 
	                              NULL, &adata, m_maxIter, 1, opts, info);//, sba_costL2, NULL);
	
	if( iters == SBA_ERROR ) 
		printf( "CBundleAdjustAdapter::DoMotionF() error\n" );

	else 
	{
		printf( "Bundle adjustment took %d iterations,\n", iters );
		printf( "Error at termination (||e||_2): %f\n", info[1] );
		printf( "Reason for terminating: %s\n\n", sba_term_string(info[6]) );
		//more info ...
		
		// get adjusted cameras
		for( int i=0; i<m_numCamera; ++i )
		{
			double* pCamParam = params +i*camParams;

			m_cam[i].SetFocalLength( *pCamParam );		
			m_cam[i].SetPose( pCamParam+1 );			
		}
	}
	
	delete [] params;
	delete [] vmask;
	delete [] adata.X;
	delete [] x;
}

