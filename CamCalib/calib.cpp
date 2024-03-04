#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/core.hpp>

using namespace std;

#include "2702camera.h"

using namespace Rebels;

#include "stereoCalib.h"
#include "RawFile.h"

Rebels::StereoCam	theCam;
RawFile rf;

int main( int argc, char **argv )
{
	theCam.bLoad("comb");


	if( !rf.bLoad( argv[1] ) ) {
		cerr << "Unable to open raw file\n";
		return 1;
	}

#if 0
	for( size_t i = 0; i < rf.nFrames(); i++ ) {
		std::vector<std::vector<Rebels::Point2d>> &vFrame = rf[i];

		printf( "Frame %d\n", i );

		for( int m = 0; m < 4; m++ ) {
			printf( "M%d\n    ", m+1);
			for( int s = 0; s < 2; s++ ) {
				Rebels::Point2d const &p = vFrame[m][s];
				printf( " (%9.3f,%9.3f)", p.u(), p.v() );
			}
			printf( "\n" );
		}
	}
#endif
	StereoCalib sc( theCam, rf.Frames() );
	sc.fCalibrate( 0.f, 0.f );
printf( "Calibrated\n" );
	sc.saveLog( "Calib-Res.log" );

	theCam.Save( "new" );
	return 0;
}
