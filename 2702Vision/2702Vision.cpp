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
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/FloatTopic.h>
#include <networktables/BooleanTopic.h>
#include <frc/geometry/Pose3d.h>


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
using namespace frc;

#include "2702camera.h"
#include "2702Vision.h"

#if INCLUDE_GUI
#include <curses.h>
#include "genio.h"
#endif

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

bool bShutdown = false;

void sighandler( int nSig )
{
	printf( "Received Signal %d\n", nSig );
	bShutdown = true;
}

void RunCam()
{
	int nRawImg = 0;
	int nRGBImg = 0;
	int nGreenImg = 0;
	char szTmp[1024];

	bool bStop = false;

	bpiCamera mycam0( "/dev/video0", 4, 0 );
	while (!bStop && !bShutdown) {
#if INCLUDE_GUI
		bool bRefresh = false;
		int ch = getch();
		switch( ch ) {
			case -1:
				break;
			case 0x1b:
				bStop = true;
				break;
			case ' ':
				clear();
				break;
			case 'r': case 'b':
				sprintf( szTmp, "bayer%04d.png", ++nRawImg );
				mycam0.vSaveImage( szTmp, 0 );
				bRefresh = true;
				break;
			case 'c':
				sprintf( szTmp, "color%04d.png", ++nRGBImg );
				mycam0.vSaveImage( szTmp, 1 );
				bRefresh = true;
				break;
			case 'g':
				sprintf( szTmp, "green%04d.png", ++nGreenImg );
				mycam0.vSaveImage( szTmp, 2 );
				bRefresh = true;
				break;
			default:
				break;
		}
		if (bRefresh) {
			mvprintw( 2, 1, "Bayer Images: %d", nRawImg );
			mvprintw( 3, 1, "Color Images: %d", nRGBImg );
			mvprintw( 4, 1, "Green Images: %d", nGreenImg );
			refresh();
		}
#endif
		std::this_thread::sleep_for( std::chrono::milliseconds(200) );
	}
}

void Usage()
{
	printf( "Usage: 2702Vision [-s]\n" );
	printf( "    -s            run as NetTables Server\n" );
	exit(1);
}

int main( int argc, char **argv )
{
	bool bNetTableServer = false;

	signal( SIGINT, sighandler );
	signal( SIGTERM, sighandler );
	signal( SIGKILL, sighandler );

	for (int n = 1; n < argc; n++) {
		if (argv[n][0] == '-') {
			switch (argv[n][1]) {
			case 's':
				bNetTableServer = true;
				break;
			default:
				Usage();
				break;
			}
		}
	}

	// start NetworkTables
	auto ntinst = nt::NetworkTableInstance::GetDefault();
	if (bNetTableServer) {
    	ntinst.StartServer();
		ntinst.SetServerTeam(2702);
	}
	else {
		fmt::print("Setting up NetworkTables client for team 2702\n");
		ntinst.StartClient4("2702Vision");
		ntinst.SetServerTeam(2702);
		ntinst.StartDSClient();

		printf( "Waiting for Network Tables to connect\n" );
		int nCount = 0;
		while( !ntinst.IsConnected() && !bShutdown ) {
			if (nCount++ >2) {
				exit(1);
			}
			printf( "." ); fflush( stdout );
			std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
		}
	}



	printf( "NetworkTables are %s\n", ntinst.IsConnected() ? "Connected" : "Not Connected" );
	auto dtA1PIDP = ntinst.GetDoubleTopic("/Preferences/ExArm/TuningParameters/Axis1 PID/P");
	auto dtsAxis1PIDP = dtA1PIDP.Subscribe(-100.);

	double dPIDP = dtsAxis1PIDP.Get();
	printf( "PIDP = %d\n", dPIDP );

//	bpiCamera mycam1( "/dev/video1", 4, 1 );

#if INCLUDE_GUI
	InitScreen();
    	nodelay( stdscr, 1 );
#endif
	RunCam();

	std::this_thread::sleep_for( std::chrono::milliseconds(2000) );

#if INCLUDE_GUI
	endwin();
#endif
	return 0;
}
