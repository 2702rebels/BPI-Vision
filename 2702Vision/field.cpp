#include <string>
#include <stdexcept>

#include <opencv2/core/core.hpp>
#include <frc/geometry/Transform3d.h>
#include <units/length.h>
#include <units/angle.h>

using namespace frc;
using namespace std;

#include "2702math.h"
#include "field.h"

static struct tag {
	int id;
	float x, y, z;
	int dir;
} const s_grTags[] =
{
	{1, 610.77,  42.190, 18.22, -1},
	{2, 610.77, 108.190, 18.22, -1},
	{3, 610.77, 174.190, 18.22, -1},
	{4, 636.96, 265.740, 27.38, -1},
	{5, 14.250, 265.740, 27.38, 1},
	{6, 40.450, 174.190, 18.22, 1},
	{7, 40.450, 108.190, 18.22, 1},
	{8, 40.450,  42.190, 18.22, 1}
 };

FieldXfrms::FieldXfrms() 
{
	units::length::meter_t m0( 0.F );
	units::angle::radian_t r0( 0.F );
	units::angle::radian_t rP( PI );
	units::angle::radian_t rP2( PI/2 );
	units::angle::radian_t rNP2( -PI/2 );

	// Rotates negative Z to positive X
	frc::Rotation3d r3dNZX( r0, rNP2, r0 );
	// rotates negative Z to pozitive Y
	frc::Rotation3d r3dNZY( rNP2, r0, r0 );
	frc::Rotation3d r3dZ180( r0, r0, rP );


	for( int i = 0; i < 8; i++ ) {
		units::length::meter_t fX = units::length::inch_t(s_grTags[i].x);
		units::length::meter_t  fY = units::length::inch_t(s_grTags[i].y);
		units::length::meter_t  fZ = units::length::inch_t(s_grTags[i].z);
//printf( "Tag %d : %8.3f %8.3f %8.3f\n", s_grTags[i].id, fX, fY, fZ );
		int nDir = s_grTags[i].dir;
		frc::Rotation3d rTotal = r3dNZX + r3dNZY;
		if (nDir == -1) {
			rTotal = rTotal + r3dZ180;
		}
		m_grxfmTags[i] = frc::Transform3d( Translation3d( fX, fY, fZ ), rTotal );
	}
}

FieldXfrms::~FieldXfrms()
{
}

void FieldXfrms::GetTagCorners( int nTag, std::vector<cv::Point3f> &vPts )
{
	if( nTag < 1 || nTag > 8 )
		throw runtime_error( string( "FieldXfrms::GetTagCorners for invalid tag\n" ) );

	
	auto t = s_grTags[nTag - 1];

	vPts.push_back( cv::Point3d( t.x, t.y - (3.0 * t.dir), t.z - 3 ) );
	vPts.push_back( cv::Point3d( t.x, t.y + (3.0 * t.dir), t.z - 3 ) );
	vPts.push_back( cv::Point3d( t.x, t.y + (3.0 * t.dir), t.z + 3 ) );
	vPts.push_back( cv::Point3d( t.x, t.y - (3.0 * t.dir), t.z + 3 ) );
}

#if 0
FieldXfrms myField;

int main( int argc, char **argv )
{
	Transform3d x3dTag( Translation3d( units::length::meter_t(2.F), units::length::meter_t(-0.464F), units::length::meter_t(1.F) ),
					  Rotation3d( units::angle::radian_t(0.F), units::angle::radian_t(0.F), units::angle::radian_t(0.F) ) );


//	Transform3d pose = x3dTag.Inverse() + myField[6];
	Transform3d pose = myField[3] + x3dTag.Inverse();

	printf( "Result: -> %8.3f %8.3f %8.3f\n", pose.X(), pose.Y(), pose.Z() );
	return 0;
}
#endif
