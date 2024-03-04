#include <vector>

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Transform2d.h>
#include <units/length.h>
#include <opencv2/core/core.hpp>

#include "2702camera.h"

#include "PoseEstimator.h"

using namespace frc;

#define CMINPACK_NO_DLL
#define __cminpack_float__
#include "cminpack.h"

int RobotPoseEstimator::Evaluate( void *p, int m, int n, const float *pfX, float *pfVals, int nFlg )
{
	RobotPoseEstimator *prpe = (RobotPoseEstimator *)p;
	prpe->Evaluate( m, n, pfX, pfVals, nFlg );
	return 0;
}

float RobotPoseEstimator::Evaluate( int m, int n, const float *pfX, float *pfVals, int nFlg )
{
	size_t
		nSample = 0,
		nSamples = m;
	double
		dfSumSqd = 0.;
	size_t nIdx = 0;

	float fTheta = pfX[0];
	float fXT = pfX[1];
	float fYT = pfX[2];
	float fZT = pfX[3];

//printf( "Evaluate: %8.5f %8.2f %8.2f\n", fTheta, fXT, fYT );
	float fSinTheta = sin(fTheta);
	float fCosTheta = cos(fTheta);


	for( size_t i = 0; i < m_vPointCoords.size(); i++ ) {
		cv::Point3f	const &p3f = m_vPointCoords[i];
		cv::Point2f const &p2f = m_vImgCoords[i];

		float fX = p3f.x * fCosTheta - p3f.y * fSinTheta + fXT; 
		float fY = p3f.x * fSinTheta + p3f.y * fCosTheta + fYT; 
		float fZ = p3f.z + fZT;

		// in field coordinate system depth is X
		float fUMeas = -fY/fX;
		float fVMeas =  fZ/fX;

//		printf( "%8.2f %8.2f %8.2f -> %8.2f %8.2f %8.2f -> %8.5f/%8.5f %8.5f/%8.5f\n", p3f.x, p3f.y, p3f.z, fX, fY, fZ, fUMeas, p2f.x, fVMeas, p2f.y );

		pfVals[nIdx++] = fUMeas - p2f.x;
		pfVals[nIdx++] = fVMeas - p2f.y;
	}

	float fMax = -10000.F;
	int nMaxPt = 0;
	for( size_t i = 0; i < nIdx; i++ )
	{
		float fErr = fabs( pfVals[i] );
		if( fErr > fMax )
		{
			fMax = fErr;
			nMaxPt = i;
		}
		dfSumSqd += pfVals[i] * pfVals[i];
	}
	if( nFlg == 1 )
	{
		char szTmp[256];
		sprintf( szTmp, "Num Pts = %d, Max = %f@%d, Chi = %f, RMS = %f", 
				 (int)nSamples, fMax, nMaxPt, dfSumSqd, sqrt( dfSumSqd / nSamples ) );
//		printf( "%s\n", szTmp );
	}
	return sqrt(dfSumSqd/nSamples);
}

RobotPoseEstimator::RobotPoseEstimator( Rebels::CamParm const &cp, std::vector<cv::Point2f> const &vImgCoords, std::vector<cv::Point3f> const &vPtCoords ) :
	m_cp( cp ),
	m_vPointCoords( vPtCoords )
{
	float fFU = m_cp.pinhole(0,0);
	float fFV = m_cp.pinhole(1,1);
	float fU0 = m_cp.pinhole(0,2);
	float fV0 = m_cp.pinhole(1,2);

	for( size_t i = 0; i < vImgCoords.size(); i++ ) {
		auto &p = vImgCoords[i];
		float fU = (p.x - fU0) / fFU;
		float fV = (p.y - fV0) / fFV;
		float fUndU;
		float fUndV;
		m_cp.Undistort( fU, fV, fUndU, fUndV );
		m_vImgCoords.push_back( cv::Point2f( fUndU, fUndV ) );
	}
}

RobotPoseEstimator::~RobotPoseEstimator()
{
}


frc::Transform2d RobotPoseEstimator::EstimatePose2D()
{
	int nEqns = m_vImgCoords.size() * 2;		// U, V errors
	int nUnk = 4;
	int info = 0;
	int lwa = nUnk * nEqns + 5 * nUnk + nEqns;

	int 	grnWA[nUnk];
	float	grfWA[lwa];
	float	fTol = 0.001;
	float	grfX[nUnk] = { 0.f };
	float	grfBest[nUnk];
	float	grfY[nEqns];
	float	fBest = 100.;


printf( "Estimating Pose!!\n" );
for( int i = 0; i < m_vPointCoords.size(); i++ ) {
	cv::Point3f const &p = m_vPointCoords[i];
	printf( "%f %f %f\n", p.x, p.y, p.z );
}
	for( int pose = 0; pose < 2; pose++ ) {
		// decide which half of field...
		if( m_vPointCoords[0].x < 27.F * 12.F ) {
			// Blue Alliance Wall
			grfX[0] = 0.f;	// Rotation about field
			grfX[1] = -15.F * 12.F;
			grfX[2] = -m_vPointCoords[0].y - 36.F + pose * 72.F;
			grfX[3] = -18.F;
		}
		else {
			grfX[0] = PI;
			grfX[1] = 15.F * 12.F;
			grfX[2] = -m_vPointCoords[0].y - 36.F + pose * 72.F;
			grfX[3] = -18.F;
		}

		int nRet = slmdif1( Evaluate, (void *)this, nEqns, nUnk, grfX, grfY, fTol, grnWA, grfWA, lwa );
		printf( "%d slmdif1 returned %d\n", pose, nRet );
		printf( "%8.2f %8.2f %8.2f %8.4f\n", grfX[1], grfX[2], grfX[3], grfX[0] );
		float fRMS = Evaluate( nEqns, nUnk, grfX, grfY, 1 );
		if( !pose || fRMS < fBest ) {
			for( int i = 0; i < nUnk; i++ ) {
				grfBest[i] = grfX[i];
			}
			fBest = fRMS;
		}
		
	}
	units::length::inch_t itX(grfBest[1]);
	units::length::inch_t itY(grfBest[2]);
	units::length::inch_t itZ(grfBest[3]);

	frc::Transform3d t3d( frc::Translation3d( itX, itY, itZ ),
							frc::Rotation3d(  units::angle::radian_t(0.F), units::radian_t(0.f), units::angle::radian_t(grfBest[0]) ) );
//							frc::Rotation3d( units::angle::radian_t(grfBest[0]), units::angle::radian_t(0.F), units::radian_t(0.f) ) );

	frc::Transform3d t3Inv = t3d.Inverse();

	printf( "->>>%8.2f %8.2f %8.2f   %8.4f %8.4f %8.4f\n", t3Inv.X().value(), t3Inv.Y().value(), t3Inv.Z().value(),
		t3Inv.Rotation().X().value(), t3Inv.Rotation().Y().value(), t3Inv.Rotation().Z().value() );
	frc::Transform2d t2d( frc::Translation2d( t3Inv.X(), t3Inv.Y() ), frc::Rotation2d( t3Inv.Rotation().Z() ) );
	return t2d;
}
