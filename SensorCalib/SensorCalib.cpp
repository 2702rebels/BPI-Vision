#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;

#include "VisLog.h"

void Usage( char const *psz )
{
	fprintf( stderr, "Usage: %s inputfile\n", psz );
	exit(1);
}

int main( int argc, char **argv )
{
	char const *pszIn = 0;

	for( int i = 1; i < argc; i++ ) {
		if (argv[i][0] == '-o') {
			Usage( argv[0] );
		}
		else if (!pszIn) {
			pszIn = argv[i];
		}
	}

	if (!pszIn) {
		Usage( argv[0] );
	}
	printf( "InFile %s\n", pszIn );

	VisionLog vl( pszIn );
	int nFrms = 0;
	cv::Mat frm;

	int32_t i32Frm;
	int nExp;

	while ((i32Frm = vl.i32ReadNext( frm, nExp )) > 0) {
		nFrms++;
	}

	printf( "%d Frames\n", nFrms );
}
