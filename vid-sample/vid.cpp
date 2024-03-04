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
#include "fbdisp.h"
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

int main( int argc, char **argv )
{
	signal( SIGINT, sighandler );

	uint32_t u32 = 0x01020304;
	uint8_t *pu8 = (uint8_t *)&u32;
	printf( "%02X%02X%02X%02X\n", pu8[0], pu8[1], pu8[2], pu8[3] );
//	auto ntinst = nt::NetworkTableInstance::GetDefault();
//	ntinst.StartClientTeam( 2702 );
//	ntinst.StartServer();
	bpiCamera mycam0( "/dev/video0", 4, 0 );
//	bpiCamera mycam1( "/dev/video1", 4, 1 );

	while( !bStop ) {
		std::this_thread::sleep_for( std::chrono::milliseconds(200) );
	}

	return 0;
}
