#include <unistd.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <curses.h>

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

#include "genio.h"

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

void RunCam()
{
	int nRawImg = 0;
	int nRGBImg = 0;
	int nGreenImg = 0;
	char szTmp[1024];

	bool bStop = false;

	bpiCamera mycam0( "/dev/video0", 4, 0 );
	while( !bStop ) {
		int ch = getch();
		switch( ch ) {
			case -1:
				break;
			case 0x1b:
				bStop = true;
				break;
			case ' ':
				break;
			case 'r': case 'b':
				sprintf( szTmp, "bayer%04d.png", ++nRawImg );
				mycam0.vSaveImage( szTmp, 0 );
				break;
			case 'c':
				sprintf( szTmp, "color%04d.png", ++nRGBImg );
				mycam0.vSaveImage( szTmp, 1 );
				break;
			case 'g':
				sprintf( szTmp, "green%04d.png", ++nGreenImg );
				mycam0.vSaveImage( szTmp, 2 );
				break;
			default:
				break;
		}
		mvprintw( 2, 1, "Bayer Images: %d", nRawImg );
		mvprintw( 3, 1, "Color Images: %d", nRGBImg );
		mvprintw( 4, 1, "Green Images: %d", nGreenImg );
		refresh();
		std::this_thread::sleep_for( std::chrono::milliseconds(200) );
	}
}

int main( int argc, char **argv )
{
	signal( SIGINT, sighandler );


//	auto ntinst = nt::NetworkTableInstance::GetDefault();
//	ntinst.StartClientTeam( 2702 );
//	ntinst.StartServer();
//	bpiCamera mycam1( "/dev/video1", 4, 1 );

	InitScreen();
    	nodelay( stdscr, 1 );
	RunCam();

	std::this_thread::sleep_for( std::chrono::milliseconds(2000) );

	endwin();
	return 0;
}
