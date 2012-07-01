#include "Camera.h"

#include <fstream>
using namespace std;

CCamera::CCamera(void)
{
	f = u = v = 0.0;
	alpha = 1.0;

	memset( rod, 0, sizeof(double)*3 );
	memset( t, 0, sizeof(double)*3 );
}

CCamera::CCamera( double f, double u, double v, double alpha, const double* rod, const double* t )
{
	this->f = f;
	this->u = u;
	this->v = v;
	this->alpha = alpha;

	if( rod )
		memcpy( this->rod, rod, sizeof(double)*3 );
	else
		memset( this->rod, 0, sizeof(double)*3 );

	if( t )
		memcpy( this->t, t, sizeof(double)*3 );
	else
		memset( this->t, 0, sizeof(double)*3 );

	UpdateProjMat();
}

CCamera::CCamera( const CCamera& cam ):
	f(cam.f), u(cam.u), v(cam.v), alpha(cam.alpha)
{
	memcpy( rod, cam.rod, sizeof(double)*3 );
	memcpy( t, cam.t, sizeof(double)*3 );

	UpdateProjMat();
}

CCamera::~CCamera(void)
{
}

CCamera& CCamera::operator=( const CCamera& cam )
{
	f = cam.f;
	u = cam.u;
	v = cam.v;
	alpha = cam.alpha;

	memcpy( rod, cam.rod, sizeof(double)*3 );
	memcpy( t, cam.t, sizeof(double)*3 );

	UpdateProjMat();

	return *this;
}

void CCamera::SetIntMatrix( const double* mat )
{
	f = mat[0];
	u = mat[2];
	v = mat[5];
	alpha = mat[4]/mat[0];
	
	UpdateProjMat();
}

// get the 3x3 intrinsic camera matrix
void CCamera::GetIntMatrix( double* mat ) const
{
	memset( mat, 0, sizeof( double )*9 );

	mat[0] = f;
	mat[2] = u;
	mat[4] = alpha*f;
	mat[5] = v;
	mat[8] = 1;	
}

void CCamera::SetExtMatrix( const double* mat )
{
	// rotation
	double R[9] = {
		mat[0], mat[1], mat[2],
		mat[4], mat[5], mat[6],
		mat[8], mat[9], mat[10]};
	CvMat matR = cvMat( 3, 3, CV_64F, R );
	CvMat matRd = cvMat( 3, 1, CV_64F, rod );
	cvRodrigues2( &matR, &matRd );

	// translation
	t[0] = mat[3];
	t[1] = mat[7];
	t[2] = mat[11];

	UpdateProjMat();
}

// get the 3x4 extrinsic camera matrix
void CCamera::GetExtMatrix( double* mat ) const
{
	double r[9];
	CvMat matR = cvMat( 3, 3, CV_64F, r );

	double rod_[3] = { rod[0], rod[1], rod[2] };
	const CvMat matRod = cvMat( 3, 1, CV_64F, rod_ );
	cvRodrigues2( &matRod, &matR );

	for( int i=0; i<3; ++i )
	{
		memcpy( mat+i*4, r+i*3, sizeof(double)*3 );
		mat[i*4+3] = t[i];
	}
}

void CCamera::UpdateProjMat()
{
	double dI[9];
	CvMat matI = cvMat( 3, 3, CV_64F, dI );	
	
	double dE[12];
	CvMat matE = cvMat( 3, 4, CV_64F, dE );
	
	GetIntMatrix( dI );
	GetExtMatrix( dE );

	//
	CvMat matP = cvMat( 3, 4, CV_64F, d );
	cvMatMul( &matI, &matE, &matP );
}

void CCamera::SetFocalLength( double f )
{
	this->f = f;
	UpdateProjMat();
}

double CCamera::GetFocalLength() const
{
	return f;
}

void CCamera::SetAlpha( double alpha )
{
	this->alpha = alpha;
	UpdateProjMat();
}

double CCamera::GetAlpha() const
{
	return alpha;
}

void CCamera::SetImageCenter( const CvPoint2D64f& pt )
{
	u = pt.x;
	v = pt.y;

	UpdateProjMat();	
}

CvPoint2D64f CCamera::GetImageCenter() const
{
	return cvPoint2D64f( u, v );
}

void CCamera::SetPose( const double* rod, const double* t )
{
	if(rod)
		memcpy( this->rod, rod, sizeof(double)*3 );

	if(t)
		memcpy( this->t, t, sizeof(double)*3 );

	UpdateProjMat();	
}

void CCamera::GetPose( double* rod, double* t ) const
{
	if(rod)
		memcpy( rod, this->rod, sizeof( double )*3 );

	if(t)
		memcpy( t, this->t, sizeof( double )*3 );
}

CvPoint2D64f CCamera::GetProjection( const CvPoint3D64f& pt ) const
{
	double x = pt.x*d[0] +pt.y*d[1] +pt.z*d[2] +d[3];
	double y = pt.x*d[4] +pt.y*d[5] +pt.z*d[6] +d[7];
	double z = pt.x*d[8] +pt.y*d[9] +pt.z*d[10]+d[11];

	return cvPoint2D64f( x/z, y/z );
}

CvPoint3D64f CCamera::GetPositionWS( CvPoint3D64f ptcs ) const
{
	double dExt[16] = { 
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1};

	CvMat matExt = cvMat( 4, 4, CV_64F, dExt );

	GetExtMatrix( dExt );

	cvInvert( &matExt, &matExt );

	// Point position in world space
	CvPoint3D64f ptws;
	ptws.x = dExt[0]*ptcs.x +dExt[1]*ptcs.y +dExt[2]*ptcs.z + dExt[3];
	ptws.y = dExt[4]*ptcs.x +dExt[5]*ptcs.y +dExt[6]*ptcs.z + dExt[7];
	ptws.z = dExt[8]*ptcs.x +dExt[9]*ptcs.y +dExt[10]*ptcs.z + dExt[11];

	return ptws;
}

void CCamera::GetRayDirectionWS( double dir[3], const CvPoint2D64f& pt ) const
{
	double dInt[9];
	GetIntMatrix( dInt );

	// the ray vector in camera space
	double vx = ( pt.x -dInt[2] )/dInt[0];
	double vy = ( pt.y -dInt[5] )/dInt[4];
	double vz = 1;

	CvPoint3D64f ptAt = GetPositionWS( cvPoint3D64f(0,0,0) );
	CvPoint3D64f ptTo = GetPositionWS( cvPoint3D64f(vx,vy,vz) );

	double len = sqrt( vx*vx +vy*vy +vz*vz );
	dir[0] = ( ptTo.x -ptAt.x )/len;
	dir[1] = ( ptTo.y -ptAt.y )/len;
	dir[2] = ( ptTo.z -ptAt.z )/len;	
}

void LoadCamera( CCamera& cam, int i )
{
	char strName[128];
	sprintf( strName, "%04d.camera", i );

	ifstream in( strName );

	double f, u, v, alpha, r[3], t[3];
	in >> f >> u >> v >> alpha;
	in >> r[0] >> r[1] >> r[2];
	in >> t[0] >> t[1] >> t[2];

	cam.SetFocalLength(f);
	cam.SetImageCenter( cvPoint2D64f( u, v ) );
	cam.SetAlpha( alpha );

	cam.SetPose( r, t );

	in.close();
}

void SaveCamera( const CCamera& cam, int i )
{
	char strName[128];
	sprintf( strName, "%04d.camera", i );

	ofstream out( strName );

	//
	out << cam.GetFocalLength() << " "
		<< cam.GetImageCenter().x << " "
		<< cam.GetImageCenter().y << " "
		<< cam.GetAlpha() << endl;

	double r[3], t[3];
	cam.GetPose( r, t );

	out << r[0] << " " << r[1] << " " << r[2] << endl;
	out << t[0] << " " << t[1] << " " << t[2] << endl;

	out.close();	
}

