#include <unistd.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTableInstance.h>

#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <string>
#include <stdexcept>

using namespace std;

#include "v4lcam-mp.h"
#include "bpicam.h"

#include <signal.h>

void printinfo( v4lcam &cam )
{
	cam.printCaps();
	cam.printStandards();
	cam.printControls();
	printf( "---------------\n" );
}

bool bStop = false;

void sighandler( int nSig )
{
	printf( "Received Signal %d\n", nSig );
	bStop = true;
}

void Usage()
{
	cerr << "Usage:  VidCapture outfile\n";
	exit( 1 );
}

int main( int argc, char **argv )
{
	char *pszFile = 0;

	for( int i = 1; i < argc; i++ ) {
		if (!pszFile) {
			pszFile = argv[i];
		}
		else {
			Usage();
		}
	}

	if( !pszFile ) {
		Usage();
	}
	signal( SIGINT, sighandler );

	bpiCamera mycam0( "/dev/video0", 4, 0, pszFile );
//	bpiCamera mycam1( "/dev/video1", 4, 1 );

	while( !bStop ) {
		std::this_thread::sleep_for( std::chrono::milliseconds(200) );
	}

	return 0;
}
