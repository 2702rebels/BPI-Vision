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

void Usage()
{
	printf( "Usage: 2702Vision [-s]\n" );
	printf( "    -s            run as NetTables Server\n" );
	exit(1);
}

int main( int argc, char **argv )
{
	bool bNetTableServer = false;

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
		while( !ntinst.IsConnected() ) {
			printf( "." ); fflush( stdout );
			std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
		}
	}



	printf( "NetworkTables are %s\n", ntinst.IsConnected() ? "Connected" : "Not Connected" );
	auto dtA1PIDP = ntinst.GetDoubleTopic("/SmartDashboard/Vision/Exposure");
	auto dtsAxis1PIDP = dtA1PIDP.Subscribe(-100.);

	
	std::this_thread::sleep_for( std::chrono::milliseconds(1000) );

	double dPIDP = dtsAxis1PIDP.Get();
	printf( "PIDP = %f\n", dPIDP );

	auto bpIsLogging = ntinst.GetBooleanTopic("/Vision/IsLogging").Publish();
	bpIsLogging.Set( false );

	std::this_thread::sleep_for( std::chrono::milliseconds(1000) );

//	bpiCamera mycam1( "/dev/video1", 4, 1 );

	return 0;
}
