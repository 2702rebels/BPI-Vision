#include <stdio.h>

#include <fstream>
#include <vector>
#include <string>

#include "2702math.h"
#include "2702camera.h"

using namespace Rebels;
using namespace std;
#include "monoCalib.h"

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
MonoCalib::MonoCalib( CamParm &cam, vector<vector<Point2d> >&calPts ) : 
	m_Cam( cam ), m_calPoints( calPts )
{
}

MonoCalib::~MonoCalib()
{
}

int MonoCalib::Evaluate( void *p, int m, int n, const float *pdfX, float *pdfVals, int nFlg )
{
	MonoCalib *pCalib = (MonoCalib *)p;
	return pCalib->Evaluate( m, n, pdfX, pdfVals, nFlg );
}

void MonoCalib::UpdateCamXfrms( const float *pfX )
{
	Quaternion qx( pfX[0], Point3d( 1.f, 0.f, 0.f ) );
	Quaternion qy(    0.F, Point3d( 0.0f, 1.0f, 0.0f ) );
	Quaternion qz( pfX[1], Point3d( 0.0f, 0.0f, 1.0f ) );

	Point3d Origin(  0.f, 0.f, pfX[2] );

#if 1
	Quaternion qc = qx * qy* qz;
#else
	Quaternion qc = qz * qy * qx;
#endif

	m_Cam.SetRotation( qc );
	m_Cam.SetOrigin( Origin );
}

int MonoCalib::Evaluate( int m, int n, const float *pdfX, float *pdfVals, int nFlg )
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

	for( size_t nPoint = 0; nPoint < m_calPoints.size(); nPoint++ )
	{
		Point2d &p1 = m_calPoints[nPoint][0];
		Point2d &p2 = m_calPoints[nPoint][1];

		Point3d pt1 = m_Cam.CalcTargetPosFromHeight( p1.u(), p1.v(), m_fFloorHeight );
		Point3d pt2 = m_Cam.CalcTargetPosFromHeight( p2.u(), p2.v(), m_fFloorHeight );

		// The first point must be taken with the midpoint of the cal stick aligned with the X axis.
//		if( nPoint == 0 )
//		{
//			Point3d mid = (pt1 + pt2)/2;
//			pdfVals[nIdx++] = mid.x();
//		}
		pdfVals[nIdx++] = (pt1 - pt2).length() - m_fBarLength;
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
		char szTmp[1024];
		sprintf( szTmp, "Num Pts = %d, Max = %f@%d, Chi = %f, RMS = %f", 
				 (int)nSamples, dfMax, nMaxPt, dfSumSqd, sqrt( dfSumSqd / nSamples ) );
		m_log.push_back( szTmp );
	}
	return 0;
}


float MonoCalib::fCalibrate( float fBarLength, float fFloorHeight )
{
	m_fBarLength = fBarLength;
	m_fFloorHeight = fFloorHeight;

	// we only get 1 distance for each pair of points
	int nEqns = m_calPoints.size();

	// need to calculate Rx, Rz, and Tz
	int nUnk = 3;
	int info = 0;
	int lwa = nUnk * nEqns + 5 * nUnk + nEqns;
	int *pnWA = new int[ nUnk ];
	float *pdfWA = new float[ lwa ];
	float dfTol = 0.000001;
	float *pfX = new float [ nUnk ];
	for( int i = 0; i < nUnk; i++ )
			pfX[i] = 0.;
	float *pfY = new float [ nEqns ];

	pfX[0] = -45.f * PI / 180.f;
	pfX[1] = 0.f;
	pfX[2] = 0.f;

	// set camera to robot transformation to unity.
	int nRet = slmdif1( Evaluate, (void *)this, nEqns, nUnk, pfX, pfY, dfTol, pnWA, pdfWA, lwa );

	for( int i = 0; i < nUnk; i++ )
	{
		char szTmp[64];
		if( i < 2 )
			sprintf( szTmp, ">> %.6f", pfX[i] * 180.F /PI );
		else
			sprintf( szTmp, ">> %.6f", pfX[i] / .0254F );
		m_log.push_back( szTmp );
		m_res.push_back( szTmp );
	}

	UpdateCamXfrms( pfX );

	double
		dfErr,
		dfSumSqd = 0.;
	size_t nIdx = 0;
	
	for( size_t nPoint = 0; nPoint < m_calPoints.size(); nPoint++ )
	{
		Point2d &p1 = m_calPoints[nPoint][0];
		Point2d &p2 = m_calPoints[nPoint][1];

		Point3d pt1 = m_Cam.CalcTargetPosFromHeight( p1.u(), p1.v(), m_fFloorHeight );
		Point3d pt2 = m_Cam.CalcTargetPosFromHeight( p2.u(), p2.v(), m_fFloorHeight );

		dfErr = (pt1 - pt2).length() - fBarLength;
		
		dfSumSqd += dfErr * dfErr;
		char szTmp[1024];
		sprintf( szTmp, "%.3f,%.3f,%.3f - %.3f,%.3f,%.3f --> %.3f", 
			pt1.x(), pt1.y(), pt1.z(), pt2.x(), pt2.y(), pt2.z(), (pt2 - pt1).length() );

		m_log.push_back( szTmp );
		m_res.push_back( szTmp );
	}
	delete [] pnWA;
	delete [] pdfWA;
	delete [] pfX;
	delete [] pfY;

	return sqrt(dfSumSqd / nEqns);
}

void MonoCalib::saveLog( char const *psz )
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
