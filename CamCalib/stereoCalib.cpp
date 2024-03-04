#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <assert.h>

#include <opencv2/core.hpp>

#include "2702math.h"
#include "2702camera.h"
#include "rxfm.h"

using namespace Rebels;
using namespace std;
#include "stereoCalib.h"

#if 0
extern "C" 
{
#define CMINPACK_NO_DLL
#define __cminpack_float__
//#include "minpack.h"
#include "cminpack.h"

int EvaluateSCam( void *p, int m, int n, const __cminpack_real__ *pdfX, __cminpack_real__ *pdfVals, int nFlg )
{
	Cstereovision2Dlg *pdlg = (Cstereovision2Dlg *)p;
	return pdlg->EvaluateStereo( m, n, pdfX, pdfVals, nFlg );
}

int EvaluateCamRot( void *p, int m, int n, const __cminpack_real__ *pfX, __cminpack_real__ *pfVals, int nFlg )
{
	Cstereovision2Dlg *pdlg = (Cstereovision2Dlg *)p;
	return pdlg->EvaluateCamRot( m, n, pfX, pfVals, nFlg );
}

};
#else
#define CMINPACK_NO_DLL
#define __cminpack_float__
#include "cminpack.h"
#endif
StereoCalib::StereoCalib( StereoCam &cam, vector<vector<vector<Point2d>>> const&vFrames, bool bAlignYAxis ) : 
	m_Cam( cam ), m_vFrames( vFrames ), m_bAlignYAxis( bAlignYAxis )
{
	
	cv::Mat_<cv::Vec3f> vRefPts( 4, 1 );
	vRefPts.at<cv::Vec3f>(0) = cv::Vec3f(   0.0f,   0.0f,  0.f );
	vRefPts.at<cv::Vec3f>(1) = cv::Vec3f( -152.4f,   0.0f,  0.f );
	vRefPts.at<cv::Vec3f>(2) = cv::Vec3f( -152.4f, -152.4f,  0.f );
	vRefPts.at<cv::Vec3f>(3) = cv::Vec3f(   0.0f, -152.4f,  0.f );

	m_vRefPts = vRefPts;
}

StereoCalib::~StereoCalib()
{
}

int StereoCalib::Evaluate( void *p, int m, int n, const float *pdfX, float *pdfVals, int nFlg )
{
	StereoCalib *pCalib = (StereoCalib *)p;
	return pCalib->Evaluate( m, n, pdfX, pdfVals, nFlg );
}

void StereoCalib::UpdateCamXfrms( const float *pfX )
{
//	Quaternion qx1( pfX[0], Point3d( 1.0f, 0.0f, 0.0f ) );
	Quaternion qx1( 0.F*PI/180.F, Point3d( 1.f, 0.f, 0.f ) );
	Quaternion qy1( pfX[1], Point3d( 0.0f, 1.0f, 0.0f ) );
	Quaternion qz1( pfX[2], Point3d( 0.0f, 0.0f, 1.0f ) );
	Quaternion qx2( pfX[3], Point3d( 1.0f, 0.0f, 0.0f ) );
	Quaternion qy2( pfX[4], Point3d( 0.0f, 1.0f, 0.0f ) );
	Quaternion qz2( pfX[5], Point3d( 0.0f, 0.0f, 1.0f ) );

	Point3d c1Origin(  pfX[6], 0.f, pfX[7] );
	Point3d c2Origin( -pfX[6], 0.f, pfX[8] );

#if 1
	Quaternion qc1 = qx1 * qy1 * qz1;
	Quaternion qc2 = qx2 * qy2 * qz2;
#else
	Quaternion qc1 = qz1 * qy1 * qx1;
	Quaternion qc2 = qz2 * qy2 * qx2;
#endif

	m_Cam.SetCamXfrms( qc1, c1Origin, qc2, c2Origin );
}

void CvtMatToQuat(Rebels::Quaternion &q, Point3d &p, cv::Mat &m) 
{
	assert( m.rows == 3 && m.cols == 4 );

	float grf[3][3];

	float fQ0 = 1.F;
	float fX = 0.F;
	float fY = 0.F;
	float fZ = 0.F;

	for( int i = 0; i < 3; i++ ) {
		for( int j = 0; j < 3; j++ ) {
			grf[i][j] = m.at<float>(i,j);
		}
	}
	float fT = grf[0][0] + grf[1][1] + grf[2][2] + 1;
	if (fT > 0.00000001F) {
		float fS = sqrt(fT) * 2.F;
		fQ0 = 0.25F * fS;
		fX = (grf[2][1] - grf[1][2]) / fS;
		fY = (grf[0][2] - grf[2][0]) / fS;
		fZ = (grf[1][0] - grf[0][1]) / fS;
	} else if (grf[0][0] > grf[1][1] &&
			 grf[0][0] > grf[2][2]) {
		float fS =
			sqrt(1.0F + grf[0][0] - grf[1][1] - grf[2][2]) *
			2.F;
		fX = 0.25F * fS;
		fY = (grf[1][0] + grf[0][1]) / fS;
		fZ = (grf[0][2] + grf[2][0]) / fS;
		fQ0 = (grf[2][1] - grf[1][2]) / fS;
	} else if (grf[1][1] > grf[2][2]) {
		float fS =
			sqrt(1.0F + grf[1][1] - grf[0][0] - grf[2][2]) *
			2.F;
		fX = (grf[1][0] + grf[0][1]) / fS;
		fY = 0.25F * fS;
		fZ = (grf[2][1] + grf[1][2]) / fS;
		fQ0 = (grf[0][2] - grf[2][0]) / fS;
	} else {
		float fS =
			sqrt(1.0F + grf[2][2] - grf[0][0] - grf[1][1]) *
			2.F;
		fX = (grf[0][2] + grf[2][0]) / fS;
		fY = (grf[2][1] + grf[1][2]) / fS;
		fZ = 0.25F * fS;
		fQ0 = (grf[1][0] - grf[0][1]) / fS;
	}

	q = Rebels::Quaternion( fQ0, fX, fY, fZ );
	p = Point3d( m.at<float>(0,3), m.at<float>(1,3), m.at<float>(2,3) );
}


int StereoCalib::Evaluate( int m, int n, const float *pdfX, float *pdfVals, int nFlg )
{
	size_t
		nSample = 0,
		nSamples = m;
	double
		dfSumSqd = 0.;
	size_t nIdx = 0;

	UpdateCamXfrms( pdfX );

//	Quaternion q( 1.0F, 0.F, 0.F, 0.F );
//	Point3d    t( 0.F, 0.F, 0.F );
//	m_stereoCam.SetCamToRobot( q, t );

	
	for( size_t nFrame = 0; nFrame < m_vFrames.size(); nFrame++ ) {
		vector<vector<Point2d>> const &vFrame = m_vFrames[nFrame];
		printf( "Frame %d\n", nFrame );
		cv::Mat_<cv::Vec3f> vMeasPts(4,1);
		for( size_t nCorner = 0; nCorner < 4; nCorner++ ) {
			vector<Point2d> const &vCorner = vFrame[nCorner];
			Point2d const &p1 = vCorner[0];
			Point2d const &p2 = vCorner[1];
			Point3d p3d;

			float fSep1 = m_Cam.fCalc3D( p1, p2, p3d );
			vMeasPts.at<cv::Vec3f>(nCorner) = cv::Vec3f( p3d.x(), p3d.y(), p3d.z() );
			printf( "%9.3f %9.3f, %9.3f %9.3f ->", p1.u(), p1.v(), p2.u(), p2.v() );
			printf( "%9.3f %9.3f %9.3f\n", p3d.x(), p3d.y(), p3d.z() );
			

			pdfVals[nIdx++] = fSep1;
		}
		cv::Mat_<float> mRes = FindRigidTransform(m_vRefPts,vMeasPts );
		Quaternion qRot;
		Point3d p3dXlat;

		CvtMatToQuat(qRot, p3dXlat, mRes);
		
		for( int nCorner = 0; nCorner < 4; nCorner++ ) {
			cv::Vec3f const &v3f = m_vRefPts.at<cv::Vec3f>(nCorner);
			Point3d p( v3f[0], v3f[1], v3f[2] );
			Point3d p3dNew = qRot * p + p3dXlat;
			cv::Vec3f &pmeas = vMeasPts.at<cv::Vec3f>(nCorner);
			printf( "  %9.3f %9.3f %9.3f -> %9.3f %9.3f %9.3f --> %9.3f %9.3f %9.3f\n",
				p3dNew.x(), p3dNew.y(), p3dNew.z(),
				pmeas[0], pmeas[1], pmeas[2],
				p3dNew.x() - pmeas[0],
				p3dNew.y() - pmeas[1],
				p3dNew.z() - pmeas[2] );
			pdfVals[nIdx++] = p3dNew.x() - pmeas[0];
			pdfVals[nIdx++] = p3dNew.y() - pmeas[1];
			pdfVals[nIdx++] = p3dNew.z() - pmeas[2];
		}
	}

	float dfMax = -10000.F;
	int nMaxPt = 0;
	for( size_t i = 0; i < nIdx; i++ )
	{
		float fErr = fabs( pdfVals[i] );
		if( fErr > dfMax )
		{
			dfMax = fErr;
			nMaxPt = i;
		}
		dfSumSqd += pdfVals[i] * pdfVals[i];
	}
	if( nFlg == 1 )
	{
		char szTmp[256];
		sprintf( szTmp, "Num Pts = %d, Max = %f@%d, Chi = %f, RMS = %f", 
				 (int)nSamples, dfMax, nMaxPt, dfSumSqd, sqrt( dfSumSqd / nSamples ) );
		printf( "%s\n", szTmp );
		m_log.push_back( szTmp );
	}
	return 0;
}


float StereoCalib::fCalibrate( float fBarLength, float fFloorHeight )
{
	char szTmp[256];
	m_fBarLength = fBarLength;
	m_fFloorHeight = fFloorHeight;

	// line separation for each target + z-height for first 4 positions (*2 targets/pos)
	int nEqns = m_vFrames.size() * 4 * 4;
	if( m_bAlignYAxis ) {
		nEqns += 1;
	}
	int nUnk = 9;
	int info = 0;
	int lwa = nUnk * nEqns + 5 * nUnk + nEqns;
	int *pnWA = new int[ nUnk ];
	float *pdfWA = new float[ lwa ];
	float dfTol = 0.000001;
	float *pfX = new float [ nUnk ];
	for( int i = 0; i < nUnk; i++ )
			pfX[i] = 0.;
	float *pfY = new float [ nEqns ];

	pfX[0] = 0.F * PI / 180.F;
	pfX[1] = 0.F;
	pfX[2] = 0.F;
	pfX[3] = 0.F * PI / 180.F;
	pfX[4] = 0.f;
	pfX[5] = 0.F;
	pfX[6] = 17.f / 2.f * 25.4f;
	pfX[7] = 0.f;
	pfX[8] = 0.f;
	//pfX[6] = 9.5f *.0254f;
	//pfX[7] = 45.25F * .0254F;
	//pfX[8] = 45.25F * .0254F;

	// set camera to robot transformation to unity.
	m_Cam.SetCamToRobot( Quaternion( 1.F, 0.F, 0.F, 0.F ), Point3d( 0.F, 0.F, 0.F ) ); 
	int nRet = slmdif1( Evaluate, (void *)this, nEqns, nUnk, pfX, pfY, dfTol, pnWA, pdfWA, lwa );

	m_res.push_back( "RMS=" );
	for( int i = 0; i < nUnk; i++ )
	{
		char szTmp[64];
		if( i < 6 )
			sprintf( szTmp, ">> %.6f", pfX[i] * 180.F /PI );
		else
			sprintf( szTmp, ">> %.6f", pfX[i] / .0254F );
		m_log.push_back( szTmp );
		m_res.push_back( szTmp );
	}

	Evaluate( nEqns, nUnk, pfX, pfY, 0 );

	double
		dfErr,
		dfSumSqd = 0.;
	size_t nIdx = 0;
	for( int i = 0; i < nEqns; i++ ) {
		dfSumSqd += pfY[i] * pfY[i];
	}

#if 0	
	for( size_t nPoint = 0; nPoint < m_left.size(); nPoint++ )
	{
		Point2d &c1p1 = m_left[nPoint][0];
		Point2d &c1p2 = m_left[nPoint][1];
		Point2d &c2p1 = m_right[nPoint][0];
		Point2d &c2p2 = m_right[nPoint][1];

		if( c1p1.u() < c1p2.u() ) {
			Point2d t = c1p1;
			c1p1 = c1p2;
			c1p2 = t;
		}
		if( c2p1.u() < c2p2.u() ) {
			Point2d t = c2p1;
			c2p1 = c2p2;
			c2p2 = t;
		}
		Point3d pt1;
		Point3d pt2;
		float fSep1 = m_Cam.fCalc3D( c1p1, c2p1, pt1 );
		float fSep2 = m_Cam.fCalc3D( c1p2, c2p2, pt2 );

		dfSumSqd += fSep1 * fSep1;
		dfSumSqd += fSep2 * fSep2;
		dfErr = (pt1 - pt2).length() - fBarLength;
		dfSumSqd += dfErr * dfErr;
		if( m_bAlignYAxis && !nPoint ) {
			Point3d mid = (pt1 + pt2)/2;
			dfSumSqd += mid.x() * mid.x();
		}
		if( nPoint < 4 ) {
			dfErr = pt1.z() - fFloorHeight;
			dfSumSqd += dfErr * dfErr;
			dfErr = pt2.z() - fFloorHeight;
			dfSumSqd += dfErr * dfErr;
		}
		sprintf( szTmp, "%.3f,%.3f,%.3f (%.3f)- %.3f,%.3f,%.3f (%.3f) --> %.3f", 
			pt1.x(), pt1.y(), pt1.z(), fSep1, pt2.x(), pt2.y(), pt2.z(), fSep2, (pt2 - pt1).length() );

		m_log.push_back( szTmp );
		m_res.push_back( szTmp );
	}
#endif
	delete [] pnWA;
	delete [] pdfWA;
	delete [] pfX;
	delete [] pfY;

	float fRMS = sqrt(dfSumSqd / nEqns);
	sprintf( szTmp, "RMS = %.4f", fRMS );
	m_res[0] = szTmp;
	return fRMS;
}

void StereoCalib::saveLog( char const *psz )
{
	ofstream of;

	of.open( psz );
	if( of.is_open() )
	{
		for( size_t i = 0; i < m_log.size(); i++ ) {
			of << m_log[i] << endl;
		}
	}
}
