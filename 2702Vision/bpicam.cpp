#include <unistd.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <cameraserver/CameraServer.h>

#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <string>
#include <stdexcept>

#include <sys/time.h>
#include <sys/resource.h>
#include <sched.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Pose3d.h>
#include <units/length.h>
#include <units/angle.h>

using namespace std;
using namespace frc;

#define LOG_DIR		"/storage/logs"
#define AUTO_GAIN	0
#define APRILTAGS	1
#define ANNOTATE	0

#define NUM_TAGS	18
#define BLUE1_IDX	(7-1)
#define BLUE2_IDX	(8-1)
#define RED1_IDX	(4-1)
#define RED2_IDX	(3-1)

#define HALF_RES	0

#if HALF_RES
#define RES_ROWS	(480/2)
#define RES_COLS	(752/2)
#else
#define RES_ROWS	480
#define RES_COLS	752
#endif

#define MAX_EXP		RES_ROWS

#define IsVisible(p, x) ((p)[(x) * 3] > -2.9e+16)

#include "bpicam.h"

#include "Bayer2RGB.h"
#include "Bayer2Grey.h"
#include "RollingAvg.h"

#include "2702math.h"

#define DESIRED_GREEN_INTENSITY	200

bpiCamera::bpiCamera( char const *pszVid, int nExposures, int nCam ) :
	v4lcam( pszVid ),
	m_nExposures( nExposures ),
	m_grpFrms( 0 ),
	m_bShutdown( false ),
	m_nCam( nCam ),
	m_nWorkersRunning( 0 ),
	m_nWorkersWaiting( 0 ),
	m_pNextFrame( 0 ),
#if DISPLAY
	m_disp( 100, 100, RES_ROWS, RES_COLS ),
#endif
	m_vl()
{
	m_cp.Load( "/home/pi/camera.cam" );
	if( m_nExposures > MAX_EXPOSURES )
		throw invalid_argument( string( "bpiCamera(): num exposures exceeds max" ) );

//	printf( "Instance:: %x\n", (uint32_t)frc::CameraServer::GetInstance() );

	auto ntinst = nt::NetworkTableInstance::GetDefault();
	nt::PubSubOptions pso = 
	{
		.structSize = sizeof(nt::PubSubOptions),
		.pollStorage = 0,
		.periodic = 0.1,
		.excludePublisher = 0,
		.sendAll = true,
		.topicsOnly = false,
		.keepDuplicates = true,
		.prefixMatch = false,
		.disableRemote = false,
		.disableLocal = false,
		.excludeSelf = false
//		.hidden = false
	};


	m_dapXYZ = ntinst.GetDoubleArrayTopic("/Vision/PoseArray").Publish( pso );
	m_dapTV = ntinst.GetDoubleArrayTopic("/Vision/TrapVector").Publish( pso );
	m_bpLogging = ntinst.GetBooleanTopic("/Vision/PiIsLogging").Publish( pso );
	m_bsIsLogging = ntinst.GetBooleanTopic("/Vision/IsLogging").Subscribe(false);
//	m_dsExposure = ntinst.GetDoubleTopic("/SmartDashboard/Vision/Exposure").Subscribe(480);
	m_bpLogging.Set( false );

#if CAMSERVER
	m_cs = frc::CameraServer::PutVideo( "test", RES_COLS, RES_ROWS );
#endif
//	bSetSelection( 1, 5, 640, 480 );
	bSetRes( RES_COLS, RES_ROWS, V4L2_PIX_FMT_SGRBG8 );

	m_u32MaxExp = MAX_EXP;
	for( int i = 0; i < nExposures; i++ )
	{
		m_gruNextExp[i] = MAX_EXP;
	}

	for( int i = 0; i < NUM_WORKERS; i++ )
	{
		m_grthWorkers[i] = std::thread( &bpiCamera::workerThreadFn, this );
	}
	while( m_nWorkersRunning < NUM_WORKERS )
		std::this_thread::sleep_for( std::chrono::milliseconds(10) );

	m_thCamera = std::thread( &bpiCamera::cameraThreadFn, this );
}

bpiCamera::~bpiCamera()
{
	m_bShutdown = true;
	while( m_nWorkersRunning )
		std::this_thread::sleep_for( std::chrono::milliseconds(10) );
	
	for( int i = 0; i < NUM_WORKERS; i++ )
		m_grthWorkers[i].join();

	m_thCamera.join();

	for( int i = 0; i < m_nExposures; i++ ) {
		printf( "Exposure %d = %u\n", i, m_gruNextExp[i] );
	}
	
	delete [] m_grpFrms;
}

void bpiCamera::UpdateExposures( uint32_t u32Exp )
{
	for( int i = 0; i < m_nExposures; i++ ) {
		m_gruNextExp[i] = u32Exp;
	}
	
}

void bpiCamera::UpdateVOffset( double dOffset )
{
	m_dVOffset = dOffset;
}

void bpiCamera::UpdateExposure( uint32_t u32Frame, uint32_t u32Exposure )
{
	if( u32Exposure > m_u32MaxExp )
		u32Exposure = m_u32MaxExp;
	else if( !u32Exposure )
		u32Exposure = 1;

	m_gruNextExp[ u32Frame % m_nExposures ] = u32Exposure;
}

bool bpiCamera::bProcessFrame( v4l2_buffer &buf )
{
	static uint32_t u32Lst = 0;

	uint32_t u32Frm = buf.sequence % m_nExposures ;

	//Grab some info from the buffer and store in the associted frame
	int nIdx = buf.index;
	m_grpFrms[nIdx].m_u32Seq = buf.sequence;
	m_grpFrms[nIdx].m_u32Exp = m_gruCurExp[u32Frm];
	m_grpFrms[nIdx].m_ts = buf.timestamp;
	m_grpFrms[nIdx].m_pu8Img = (uint8_t *)m_grBuffers[nIdx][0].m_pMem;

	if( u32Lst +1 != buf.sequence) {
//		printf( "Missed frame(s) %u - %u\n", u32Lst, buf.sequence );
	}
	u32Lst = buf.sequence;
#if 0
	struct timespec tsCur;
	struct timeval tvCur;
	struct timeval tvDelta;
	clock_gettime(CLOCK_MONOTONIC, &tsCur );
	TIMESPEC_TO_TIMEVAL( &tvCur, &tsCur );
	timersub( &tvCur, &buf.timestamp, &tvDelta );
	
	printf( "%d / %d   -> %d / %d = %d\n", buf.timestamp.tv_sec, buf.timestamp.tv_usec, tvCur.tv_sec, tvCur.tv_usec, tvDelta.tv_usec );
#endif
	// update the exposure for the next frame to be exposed...
	// for the mt9v032, exposure has to be set 3 frames in adavance...
	//	frame n+1 has already been exposed, and is about to be transferred
	// 	frame n+2 exposure time has already been loaded
	//  frame n+3 will be the frame affected by an updated exposure time
	// this may need to be tweaked for other sensors.
	uint32_t u32UpdateFrm = (buf.sequence + 3) % m_nExposures;
	uint32_t u32NextExp = m_gruNextExp[ u32UpdateFrm ];

#if !AUTO_GAIN
	bSetExposure( u32NextExp );
#endif
	m_gruCurExp[ u32UpdateFrm ] = u32NextExp;

	// If there is a worker thread waiting, queue this frame
	// Return false so buffer is not automatically re-queued
	return QueueWork( buf );
}

bool bpiCamera::bStart()
{
	m_grpFrms = new bpiFrame[nBuffers()];

	for( int i = 0; i < nBuffers(); i++ )
	{
		m_grpFrms[i].m_nBuffer = i;
		m_grpFrms[i].m_nRows = m_nRows;
		m_grpFrms[i].m_nCols = m_nCols;
		m_grpFrms[i].m_pu8Img = (uint8_t *)m_grBuffers[i][0].m_pMem;
	}
	return v4lcam::bStart();
}

bool bpiCamera::bStop()
{
	return v4lcam::bStop();
}

// Processing complete for frame, re-queue buffer to the driver
void bpiCamera::FrameComplete( bpiFrame *pFrame )
{
	v4l2_buffer buf;
	v4l2_plane planes[3];

	int nBuff = pFrame->m_nBuffer;
	memset( &buf, 0, sizeof(buf) );
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = nBuff;
	buf.m.planes = planes;
	buf.length = m_nPlanes;
	for( int j = 0; j < m_nPlanes; j++ )
	{
		buf.m.planes[j].bytesused = m_grBuffers[nBuff][j].m_nSize;
	}

	if( xioctl( VIDIOC_QBUF, &buf ) == -1 )
	{
		throw runtime_error( string("Failed queuing video buffer") );
	}
}

void bpiCamera::cameraThreadFn()
{
	printf( "Nice: %d\n", nice(-5) );

#if AUTO_GAIN
	bSetAutoGain( true );
	bSetAutoExposure( true );
#else
	bSetAutoExposure( false );
	bSetAutoGain( false );
	bSetGain( 64 );
#endif
	bStart();

	// As long as there are workers waiting on data,
	// keep sending it to them.
	while( m_nWorkersRunning ) {
		WaitForFrame();
	}

	bStop();
}

bool bpiCamera::QueueWork( v4l2_buffer const &buff )
{
	std::unique_lock<std::mutex> l( m_queueMutex );
	if( m_pNextFrame )
	{
		printf( "Discarding %lu\n", buff.sequence );
		return true;
	}

	m_pNextFrame = m_grpFrms + buff.index;
	if( m_nWorkersWaiting )
		m_queueCV.notify_one();
	
	return false;
}

void bpiCamera::SetThreadPriority( int policy, int pri )
{
#if 0
	sched_param parm = {
		.sched_priority = pri,
	};

	if( sched_setscheduler( 0, policy, &parm ) )
	{
		printf( "setschedparam->%d: %s\n", errno, strerror(errno) );
		throw runtime_error( string("Unable to set thread priority") );
	}
#endif
#if 0
	sched_param parm = {
		.sched_priority = pri,
	};

	if( sched_getparam( 0, &parm ) ) {
		printf( "getparam: %d:%s\n", errno, strerror(errno) );
	}
	else {
		printf( "priority: %d\n", parm. sched_priority );
	}
#endif
#if 0
	struct sched_attr attr = {
		.size = sizeof(attr),
	};

	if( sched_getattr( 0, &attr, sizeof(attr), 0 ) ) {
		printf( "getattr: %d:%s\n", errno, strerror(errno) );
	}
	else {
		printf( "size:		%d\n", attr.size );
		printf( "policy:	%d\n", attr.sched_policy );
		printf( "flags:		%x\n", attr.sched_flags );
		printf( "nice:		%d\n", attr.sched_nice );
		printf( "priority:  %d\n", attr.sched_priority );
	}

#endif
}

bpiFrame *bpiCamera::waitForWork()
{
	std::unique_lock<std::mutex> l( m_queueMutex );
	m_nWorkersWaiting++;
	m_queueCV.wait( l, [&] { return m_pNextFrame; } );

	m_nWorkersWaiting--;

	// get the pointer to the new frame to be processed
	// and reset it to 0 so no other threads try to process it
	bpiFrame *pFrm = m_pNextFrame;
	m_pNextFrame = 0;
	return pFrm;
}

#if 0
// The camera returns colour data in the Raw Bayer GRBG Format where
// each pixel has a filter and is only sensitive to Red, Green or Blue
// light.
// The first video line has pixels  GRGRGR...GR
// the second video line has pixels BGBGBG...BG
// This algorithm takes the average value of the Green pixels from each
// 2x2 block and stores that value as a "grey scale" pixel value.
// The resulting image has 1/2 the number of rows & columns, so a
// 640x480 input image becomes a 320x240 image
void bpiCamera::procGreenFrame( bpiFrame *pFrm, cv::Mat &mat, uint8_t &uMax )
{
	uint8_t	const *p = pFrm->pu8Image();
	uchar *ppix = mat.ptr(0,0);
	int nRows = mat.rows;
	uMax = 0;

	for( int r = 0; r < nRows; r++, p += 2 * m_nCols )
	{
		for( int c = 0; c < m_nCols; c+=2 )
		{
#if 0
			// for RGGB format
			uint16_t ug = ((uint16_t)p[c + 1] + p[c + m_nCols]) / 2;
			*ppix++ = ug;
#else
			// for GRBG format
			uint16_t ug = ((uint16_t)p[c] + p[c + m_nCols + 1]) / 2;
			*ppix++ = ug;
#endif
			if( ug > uMax )
				uMax = ug;
		}
	}
}
#else
// The camera returns colour data in the Raw Bayer GRBG Format where
// each pixel has a filter and is only sensitive to Red, Green or Blue
// light.
// The first video line has pixels  GRGRGR...GR
// the second video line has pixels BGBGBG...BG
// This algorithm takes the average value of the Green pixels from each
// 2x2 block and stores that value as a "grey scale" pixel value.
// The resulting image has 1/2 the number of rows & columns, so a
// 640x480 input image becomes a 320x240 image
//
// The algorithm above takes ~28 milliseconds to convert a Bayer image to
// a grey scale image. While the 4 core Banana Pi M2 Berry can easily keep
// up with a single camera at 60Hz, using 2 cameras present some challenges
//
// This version of the algorithm accounts for the fact that the ARM processor
// accesses memory 32bits at a time.
//
// Setting up pointers to the start of the 1st and 2nd video lines
// and reading the pixel data 32 bits (4 bytes) at a time reduces the
// number of memory cycles.
// 32 bit values read from the first line will be of the format
//   r2g2r1g1 where
//     r2 is the 2nd red pixel
//     g2 is the 2nd green pixel
//     r1 is the 1st red pixel
//     g1 is the 1st green pixel
// And 32 bit values from the 2nd line will be of the format
//   g2b2g1b1 where
//     b2 is the 2nd blue pixel
//     b1 is the 1st blue pixel
//
// This version of the algorithm takes ~13 milliseconds to run
//
void bpiCamera::procGreenFrame( bpiFrame *pFrm, cv::Mat &mat, uint8_t &uMax )
{
	uint32_t const *p1 = (uint32_t const *)pFrm->pu8Image();
	uint32_t const *p2;
	uint32_t *ppix = (uint32_t *)mat.ptr(0,0);
	int nRows = mat.rows;
	int nCols = mat.cols;
	uMax = 0;

	for( int r = 0; r < nRows; r++ )
	{
		p2 = (uint32_t const *)(((uint8_t const *)p1) + m_nCols);
		for( int c = 0; c < nCols; c+= 4 )
		{
			uint32_t sum;
			uint32_t pix;

			// read next 32 bit values from lines 1 & 2
			// shift the data for the 2nd line by 8 to get the green pixels to line up
			// mask out the red and blue pixel values and add the 2 32bit values
			// together and shift right by 1 to "divide by 2" and get the average
			// for the green pixels in bits 23:16 and 7:0
			sum  = (((*p1++     ) & 0x00ff00ff) + ((*p2++ >> 8) & 0x00ff00ff)) >> 1; 

			// now shift the average Green values and place them in the correct place
			// in the 32 bit output buffer ... this fills the lower 16 bits of the
			// 32 bit output buffer
			pix = (sum >> 8) & 0x0000ff00;
			sum = sum & 0xff;

			// track the brightest pixel (only check one of the 2 pixels...
			// the targes we're looking for will be at least 2 pixels wide)
			if( sum > uMax )
				uMax = sum;
			pix |= sum;

			// read the next 32 bit values as before
			sum  = (((*p1++     ) & 0x00ff00ff) + ((*p2++ >> 8) & 0x00ff00ff)) >> 1; 

			// fill the upper 16 bits of the output buffer
			pix |= (sum << 8) & 0xff000000;
			sum = sum & 0xff;

			// track the brightest pixel again.
			if( sum > uMax )
				uMax = sum;
			pix |= (sum << 16);

			// store the resulting 4 grey-scale pixel values as a single 32 bit operation
			*ppix++ = pix;
		}
		p1 = p2;
	}
}
#endif

void bpiCamera::procColorFrame( bpiFrame *pFrm, cv::Mat &mat )
{
	uint8_t	const *p = pFrm->pu8Image();
	int nRows = mat.rows;
	int nCols = mat.cols;
	uchar *ppix = mat.ptr(0,0);
	uint8_t ug;
	uint8_t const *p2 = p + m_nCols;
	for( int r = 0; r < nRows; r++, p += (m_nCols), p2 += (m_nCols) )
	{
		for( int c = 0; c < nCols; c++ )
		{
#if 0
//					ug = ((uint16_t)p[c + 1] + p[c + m_nCols]) / 2;
			*ppix++ = *p++;
			ug = ((uint16_t)*p++ + *p2++)/2;
			*ppix++ = ug;
			*ppix++ = +p2++;
#else
//				ug = ((uint16_t)p[c] + p[c + m_nCols + 1]) / 2;
//				uint8_t ug = p[c];
			*ppix++ = *p2++;
			ug = ((uint16_t)*p++ + *p2++) / 2;
			*ppix++ = ug;
			*ppix++ = *p++;
			p += 2;
			p2 += 2;
#endif
		}
	}
}


void bpiCamera::vProcTargetPoints( double *pdTags )
{
#if 1
	if( IsVisible(pdTags, BLUE1_IDX) && IsVisible(pdTags, BLUE2_IDX)) {
		double dZ1 = pdTags[BLUE1_IDX * 3 + 2];
		double dZ2 = pdTags[BLUE2_IDX * 3 + 2];
		double dX = pdTags[BLUE2_IDX * 3 + 0] - pdTags[BLUE1_IDX * 3 + 0];
		double dZ = dZ2 - dZ1;

//		printf( "%.l4f  %.4lf\n", dX, dZ );
		// Calc dot product of Vector (dX,dZ) with (1,0) and then c
		double fDotProd = dX;
		double dMag = sqrt( dX * dX + dZ * dZ );
		double fCosTheta = dX/dMag;
		double fTheta = acos( fCosTheta );
		if( dZ2 < dZ1)
			fTheta *= -1;

//		printf( "Theta = %lf\n", fTheta * 180. / PI );
		double dRotX = -m_dVOffset * -sin(fTheta);
		double dRotZ = -m_dVOffset * fCosTheta;
//printf( "%.3f  %.3f\n", dRotX, dRotZ );
		pdTags[17*3 + 0] = pdTags[BLUE1_IDX * 3 + 0] + dRotX;
		pdTags[17*3 + 1] = pdTags[BLUE1_IDX * 3 + 1];
		pdTags[17*3 + 2] = pdTags[BLUE1_IDX * 3 + 2] + dRotZ;
	}
	else if( IsVisible(pdTags,RED1_IDX) && IsVisible(pdTags,RED2_IDX)) {
		double dZ1 = pdTags[RED1_IDX * 3 + 2];
		double dZ2 = pdTags[RED2_IDX * 3 + 2];
		double dX = pdTags[RED2_IDX * 3 + 0] - pdTags[RED1_IDX * 3 + 0];
		double dZ = dZ2 - dZ1;

//		printf( "%.l4f  %.4lf\n", dX, dZ );
		// Calc dot product of Vector (dX,dZ) with (1,0) and then c
		double fDotProd = dX;
		double dMag = sqrt( dX * dX + dZ * dZ );
		double fCosTheta = dX/dMag;
		double fTheta = acos( fCosTheta );
		if( dZ2 < dZ1)
			fTheta *= -1;

//		printf( "Theta = %lf\n", fTheta * 180. / PI );
		double dRotX = -m_dVOffset * -sin(fTheta);
		double dRotZ = -m_dVOffset * fCosTheta;
//printf( "%.3f  %.3f\n", dRotX, dRotZ );
		pdTags[16*3 + 0] = pdTags[RED1_IDX * 3 + 0] + dRotX;
		pdTags[16*3 + 1] = pdTags[RED1_IDX * 3 + 1];
		pdTags[16*3 + 2] = pdTags[RED1_IDX * 3 + 2] + dRotZ;
	}
#endif
}

void bpiCamera::UndistortCorners( frc::AprilTagDetection const *pDet )
{
	double grdUOrg[8];
	double grdU[8];
					
	for( int c = 0; c < 4; c++ ) {
		auto p = pDet->GetCorner( c );
						
		float fU = p.x;
		float fV = p.y;

		float fUU; //Undistorted U
		float fUV; //Undistorted V
		grdUOrg[c * 2]     = p.x;
		grdUOrg[c * 2 + 1] = p.y;
		m_cp.UndistortPixels( fU, fV, fUU, fUV );
//		printf( "%8.4f  %8.4f -> %8.4f  %8.4f       ", fU, fV, fUU, fUV );
		grdU[c * 2]     = fUU;
		grdU[c * 2 + 1] = fUV;

	}
//	printf( "\n" );

	frc::AprilTagDetection *prwDet = (frc::AprilTagDetection *)pDet;
	prwDet->SetCorners( grdU );
}

void bpiCamera::workerThreadFn()
{
	int nRows = m_nRows/2;
	int nCols = m_nCols/2;
	cv::Mat green( nRows, nCols, CV_8UC1 );
	cv::Mat color( nRows, nCols, CV_8UC3 );
	cv::Mat blurOut;
	cv::Mat ThreshOut;

	std::unique_lock<std::mutex> l( m_queueMutex );
	m_nWorkersRunning++;
	printf( "Worker %d started\n", m_nWorkersRunning );
	printf( "sch: %d\n", sched_getscheduler(0) );
	printf( "pri: %d\n", getpriority(PRIO_PROCESS, 0) );
	l.unlock();

	Rebels::Quaternion qx( 17. * PI / 180., Rebels::Point3d(1.0, 0.0, 0.0) );
	Rebels::Quaternion qy( -3. * PI / 180., Rebels::Point3d(1.0, 0.0, 0.0) );

/*
[1051.219446928723, 0, 377.1620448386336;
 0, 1050.572091925211, 251.279222982147;
  0, 0, 1]

[525.4029379958, 0, 188.4636607918143;
 0, 525.2278614011756, 125.5839478004696;
  0, 0, 1]
*/

#if APRILTAGS
#if 0
#if HALF_RES
	AprilTagPoseEstimator::Config atPEstConf = {
		.tagSize = units::length::inch_t(6.5),
		.fx = 525.4029379958,
		.fy = 525.2278614011756,
		.cx = 188.4636607918143,
		.cy = 125.5839478004696
	};
#else
	AprilTagPoseEstimator::Config atPEstConf = {
		.tagSize = units::length::inch_t(6.5),
		.fx = 1051.219446928723,
		.fy = 1050.572091925211,
		.cx = 377.1620448386336,
		.cy = 251.279222982147
	};
#endif
#else
	AprilTagPoseEstimator::Config atPEstConf = {
		.tagSize = units::length::inch_t(6.5),
		.fx = 989.1252671735302,
		.fy = 987.199670878255,
		.cx = 386.5098017294056,
		.cy = 253.1852800724749
	};
	
#if 0
	double grdLens[9] = { 989.1252671735302, 0, 386.5098017294056, 0, 987.199670878255, 253.1852800724749, 0, 0, 1 };
	double grdDist[5] = { -0.4014696920214131, 0.3629166855537516, 0.001023238604682675, 0.001668387406228825, -0.4990975743228576 };
	cv::Mat matLens( 3, 3, cv::DataType<double>::type, grdLens );
	cv::Mat matDist( 5, 1, cv::DataType<double>::type, grdDist );
	std::vector<cv::Point3f> vObj;
	vObj.push_back( cv::Point3f( -3.25 * 0.0254,  3.25 * 0.0254, 0 ) );
	vObj.push_back( cv::Point3f(  3.25 * 0.0254,  3.25 * 0.0254, 0 ) );
	vObj.push_back( cv::Point3f(  3.25 * 0.0254, -3.25 * 0.0254, 0 ) );
	vObj.push_back( cv::Point3f( -3.25 * 0.0254, -3.25 * 0.0254, 0 ) );
#endif

#endif
	AprilTagPoseEstimator atPEst( atPEstConf );

    frc::AprilTagDetector       atd;
	auto cfg = atd.GetConfig();
#if HALF_RES
	cfg.quadDecimate = 1;
#else
	cfg.quadDecimate = 4;
#endif
	atd.SetConfig( cfg );
	atd.AddFamily("tag36h11", 0);
#endif

	RollingAverage raX(10);
	RollingAverage raY(10);
	RollingAverage raZ(10);

	while( !m_bShutdown )
	{
		bpiFrame *pFrm = waitForWork();

		uint32_t u32Frm = pFrm->u32Sequence() % m_nExposures;

		cv::Mat raw( m_nRows, m_nCols, CV_8UC1, (void *)(pFrm->pu8Image()), m_nCols );

#if APRILTAGS
		AprilTagDetector::Results detections = atd.Detect(pFrm->nCols(), pFrm->nRows(), (uint8_t *)pFrm->pu8Image() );
		int nTags = 0;
		double grdCoords[3 * NUM_TAGS];
		double grdTrap[3];
		for( int i = 0; i < NUM_TAGS * 3; i++ ) {
			grdCoords[i] = -3e16;
		}
		for( int i = 0; i < 3; i++ ) {
			grdTrap[i] = -3e16;
		}
		for( AprilTagDetection const *det : detections ) {
			int nId = det->GetId() -1;
			nTags++;

			UndistortCorners( det );
			
			Transform3d pose = atPEst.Estimate( *det );
			Rebels::Point3d pt( pose.X().value(), pose.Y().value(), pose.Z().value() );

			Rebels::Point3d pn = qx * pt;
			pn = qy * pn;

			pn -= Rebels::Point3d( 0., 26 * .0254, 0. );

			grdCoords[nId * 3 + 0] = pn.x();
			grdCoords[nId * 3 + 1] = pn.y();
			grdCoords[nId * 3 + 2] = pn.z();

			if (nId >= 11 && nId <= 16) {
				Transform3d p3dTrap = pose.Inverse();
				grdTrap[0] = p3dTrap.X().value();
				grdTrap[1] = p3dTrap.Y().value();
				grdTrap[2] = p3dTrap.Z().value();
			}
//			printf( "Have Tag: %d (%8u), %.3f, %.3f, %.3f\n", det->GetId(), pFrm->u32Sequence(), raX.fAddVal(pose.X().value() / 0.0254), raY.fAddVal(pose.Y().value() / 0.0254), raZ.fAddVal(pose.Z().value() / 0.0254) );
//			printf( "        : %d (%8u), %.3f, %.3f, %.3f\n", det->GetId(), pFrm->u32Sequence(), pt.x() / 0.0254, pt.y() / 0.0254, pt.z() / 0.0254 );
//			printf( "        : %d (%8u), %.3f, %.3f, %.3f\n", det->GetId(), pFrm->u32Sequence(), pn.x() / 0.0254, pn.y() / 0.0254, pn.z() / 0.0254 );
//			printf( "Have Tag: %d %u\n", det->GetId(), pFrm->u32Sequence() );
#if ANNOTATE
			for( int c = 0; c < 4; c++ ) {
				auto p = det->GetCorner( c );

				cv::Point cp1( p.x, p.y );
				auto p2 = det->GetCorner( (c + 1) & 3);

				cv::Point cp2;

				cp2.x = p.x + (p2.x - p.x) * (c + 1) / 4;
				cp2.y = p.y + (p2.y - p.y) * (c + 1) / 4;

				cv::line( raw, cp1, cp2, cv::Scalar( 255 ), 2 );
			}
#endif

		}
		if (!nTags) {
//			printf( "%d Tags Found\n", nTags );
		}
		{
			struct timespec tsCur;
			struct timeval tvCur;
			struct timeval tvDelta;

			vProcTargetPoints( grdCoords );
			// Get current time since power on
			clock_gettime(CLOCK_MONOTONIC, &tsCur );
			TIMESPEC_TO_TIMEVAL( &tvCur, &tsCur );

			// Calculate delta between when frame was catured and now.
			timersub( &tvCur, pFrm->ts(), &tvDelta );

			// Get current NetTables time and subtract off the delta
			int64_t time = nt::Now();
			time -= tvDelta.tv_usec;
				
//			printf( "%lld\n", time );
			
			m_dapXYZ.Set( grdCoords, time );
			m_dapTV.Set( grdTrap, time );
			auto ntinst = nt::NetworkTableInstance::GetDefault();
			ntinst.Flush();
//			printf( "%12.8f  %lld  %d\n", grdCoords[18], time, tvDelta.tv_usec );
		}
#endif

#if DISPLAY
		m_disp.Display( raw, (u32Frm & 2) * (RES_COLS/2) + m_nCam * RES_COLS*2, (u32Frm & 1) * (RES_ROWS) );
#endif

#if CAMSERVER
		if( m_mtxCS.try_lock() ) 
		{
			m_cs.PutFrame( raw );
			m_mtxCS.unlock();
		}
#endif
//		auto nExp = m_dsExposure.Get();
//printf( "Exp = %d\n", (int)nExp );
//		UpdateExposure( pFrm->u32Sequence(), nExp );
		
		if( m_mtxLog.try_lock() ) 
		{
			if (m_bLoggingDone) {
			}
			else if( m_vl.bIsOpen()) {
				m_vl.bWriteRaw( raw, pFrm->u32Sequence(), pFrm->u32Exposure() );
				if (time(0) > m_tmLogEnd) {
					m_vl.vClose();
					m_bLoggingDone = true;
					printf( "Finished Logging\n" );
				}
			}
			else {
				auto bStart = m_bsIsLogging.Get();
//				printf( "Logging = %d\n", bStart);
//				bool bStart = false;

				if( bStart ) {
					try {
						char szTmp[128];
						for (int i = 0; ; i++ )
						{
							snprintf( szTmp, sizeof(szTmp), "%s/cam%04d.log", LOG_DIR, i );
							if (access( szTmp, F_OK ) == -1) 
								break;
						}
						printf( "Opening log file at %s\n", szTmp );
						m_vl.vOpen( szTmp, true );
						m_tmLogEnd = time(0) + 160;
					}
					catch( exception &e ) {
						m_bLoggingDone = true;
						printf( "Unable to start logging: %s\n", e.what() );
					}
				}
					
			}
			m_mtxLog.unlock();
		}

		FrameComplete( pFrm );
	}
	l.lock();
	printf( "Worker %d stopping\n", m_nWorkersRunning );
	m_nWorkersRunning--;
	l.unlock();
}

