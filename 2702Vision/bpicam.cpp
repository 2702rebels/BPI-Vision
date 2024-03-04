#include <unistd.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <iostream>

#include <cameraserver/CameraServer.h>

#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Pose3d.h>
#include <units/length.h>
#include <units/angle.h>

#include "2702camera.h"
#include "2702Vision.h"

#include "Bayer2RGB.h"

#include "PoseEstimator.h"

#if INCLUDE_GUI
#include <curses.h>
#endif

using namespace std;
using namespace frc;

#include "v4lcam-mp.h"
#include "fbdisp.h"
#include "bpicam.h"
#include "field.h"

#include "rxfm.h"

#define OPTIMIZE_WINDOW 1
#define USE_DISTORTION 0
#define DESIRED_GREEN_INTENSITY	200

static FieldXfrms myField;

#define uRes 752
#define vRes 480
#define skipMargin	50

#define RobotXOffset 7.f
#define RobotZOffset 7.f

#define array_size( a ) (sizeof(a)/sizeof(a[0]))

class RollingAverage
{
	float	m_grfValues[4] = {0.F};
	int		m_nIdx = 0;

public:
	RollingAverage()
	{
	}
	~RollingAverage()
	{
	}

	float fAddValue( float fNew )
	{
		m_grfValues[m_nIdx] = fNew;
		m_nIdx = (m_nIdx + 1) % array_size(m_grfValues);

		float fSum = 0.f;
		for( int i = 0; i < array_size(m_grfValues); i++ ) {
			fSum += m_grfValues[i];
		}
		return fSum / array_size(m_grfValues);
	}

	float fRange() const {
		float fMin = m_grfValues[0];
		float fMax = fMin;
		
		for( int i = 1; i < array_size(m_grfValues); i++ ) {
			float fCur = m_grfValues[i];
			if( fMin > fCur )
				fMin = fCur;
			else if( fMax < fCur )
				fMax = fCur;
		}
		return fMax - fMin;
	}
};

RollingAverage raX;
RollingAverage raY;
RollingAverage raR;

bpiCamera::bpiCamera( char const *pszVid, int nExposures, int nCam ) :
	v4lcam( pszVid ),
	m_nSkipRows( 25 ),
	m_nActiveRows( 350 ),
	m_nExposures( nExposures ),
	m_grpFrms( 0 ),
	m_bShutdown( false ),
	m_nCam( nCam ),
	m_nWorkersRunning( 0 ),
	m_nWorkersWaiting( 0 ),
	m_pNextFrame( 0 ),
	m_disp( 10, 10, vRes/2, uRes/2 )
{
	char szTmp[1024];
#if !defined(HOME_DIR)
	char const *pszHome = getenv( "HOME" );
#else
	char const *pszHome = HOME_DIR;
#endif

	if (!pszHome)
		throw runtime_error( string( "bpiCamera(): HOME environment variable not set") );

	snprintf( szTmp, sizeof(szTmp), "%s/c%d.cam", pszHome, nCam );
	if (!m_cp.Load( szTmp ))
		throw runtime_error( string( "bpiCamera(): Camera Parameter file not found" ) );

//printf( "fx = %f\nfy = %f\ncx = %f\ncy = %f\n", m_cp.pinhole( 0, 0 ), m_cp.pinhole( 1, 1 ), m_cp.pinhole( 0, 2 ), m_cp.pinhole( 1, 2 ) );
	m_atd.AddFamily("tag16h5", 0);
	if( m_nExposures > MAX_EXPOSURES )
		throw invalid_argument( string( "bpiCamera(): num exposures exceeds max" ) );

	AprilTagFieldLayout afl = LoadAprilTagLayoutField( AprilTagField::k2023ChargedUp );
//	printf( "Instance:: %x\n", (uint32_t)frc::CameraServer::GetInstance() );

	auto ntinst = nt::NetworkTableInstance::GetDefault();
	m_dapXYR = ntinst.GetDoubleArrayTopic("/Vision/PoseArray").Publish();
	m_dapTVs = ntinst.GetDoubleArrayTopic("/Vision/TargetVectors").Publish();
	m_bpLogging = ntinst.GetBooleanTopic("/Vision/PiIsLogging").Publish();
	m_bsIsLogging = ntinst.GetBooleanTopic("/Vision/IsLogging").Subscribe(false);
	m_bpLogging.Set( false );

	units::length::meter_t m0( 0.F );
	units::angle::radian_t r0( 0.F );
	units::angle::radian_t rP2( PI/2 );

	Rotation3d r3dTot = Rotation3d( r0, r0, rP2 ) + Rotation3d( rP2, r0, r0 );

	m_x3CamToRobot = Transform3d( Translation3d( 0_in, 0_in, 0_in ), r3dTot );
	m_p3d = Pose3d( Translation3d( units::length::inch_t(0.0), units::length::inch_t(0.0), units::length::inch_t(0.0) ),
					Rotation3d( units::angle::radian_t(0.0), units::angle::radian_t(0.0), units::angle::radian_t(0.0)) );

	//annotatedOutput = frc::CameraServer::PutVideo( "test", 160, 120 );
	bSetSelection( 1, 5, uRes, vRes );
	bSetRes( uRes, vRes, V4L2_PIX_FMT_SGRBG8 );

	m_outputStream = CameraServer::PutVideo("Vision", m_nCols, m_nRows);
//	m_annotatedStream = CameraServer::PutVideo("annotated", 640, 480);
	m_u32MaxExp = 480;
	m_fdLog = open( "cam.log", O_CREAT | O_RDWR, 0660 );
	for( int i = 0; i < nExposures; i++ )
	{
		m_gruNextExp[i] = 480;
	}

#if defined(LOG_DIR1)
	for (int i = 0; ; i++ )
	{
		snprintf( szTmp, sizeof(szTmp), "%s/cam%04d.log", LOG_DIR, i );
		if (access( szTmp, F_OK ) == -1) 
			break;
	}
	//m_fp = fopen( szTmp, "w" );
	m_fp = 0;

#endif
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
	if( m_fdLog >= 0 )
		close( m_fdLog );
	
#if defined(LOG_DIR)
	if( m_fp )
			fclose( m_fp );
#endif	
	delete [] m_grpFrms;
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
	uint32_t u32Frm = buf.sequence % m_nExposures ;

	//Grab some info from the buffer and store in the associted frame
	int nIdx = buf.index;
	m_grpFrms[nIdx].m_u32Seq = buf.sequence;
	m_grpFrms[nIdx].m_u32Exp = m_gruCurExp[u32Frm];
	m_grpFrms[nIdx].m_ts = buf.timestamp;
	m_grpFrms[nIdx].m_pu8Img = (uint8_t const *)m_grBuffers[nIdx][0].m_pMem;

	// update the exposure for the next frame to be exposed...
	// for the mt9v032, exposure has to be set 3 frames in adavance...
	//	frame n+1 has already been exposed, and is about to be transferred
	// 	frame n+2 exposure time has already been loaded
	//  frame n+3 will be the frame affected by an updated exposure time
	// this may need to be tweaked for other sensors.
	uint32_t u32UpdateFrm = (buf.sequence + 3) % m_nExposures;
	uint32_t u32NextExp = m_gruNextExp[ u32UpdateFrm ];

	bSetExposure( u32NextExp );
	m_gruCurExp[ u32UpdateFrm ] = u32NextExp;
	if( !u32Frm )
		m_bSave = !m_bSave;


	// If there is a worker thread waiting, queue this frame
	// Return false so buffer is not automatically re-queued
	return QueueWork( buf );


#if 0
	if( m_fdLog >= 0 && m_bSave )
	{
		uint32_t mkr = 0xDEADBEEF;
		write( m_fdLog, &mkr, sizeof(mkr) );
		write( m_fdLog, &buf, sizeof(buf) );
		for( int j = 0; j < buf.length; j++ )
		{
			printf( "%6d (%d)\n", buf.sequence, buf.m.planes[j].length );
			write( m_fdLog, &buf.m.planes[j].length, sizeof(buf.m.planes[j].length) );
			write( m_fdLog, m_grBuffers[buf.index][j].m_pMem, buf.m.planes[j].length );
		}
	}
#endif
}

bool bpiCamera::bStart()
{
	m_grpFrms = new bpiFrame[nBuffers()];

	for( int i = 0; i < nBuffers(); i++ )
	{
		m_grpFrms[i].m_nBuffer = i;
		m_grpFrms[i].m_nRows = m_nRows;
		m_grpFrms[i].m_nCols = m_nCols;
		m_grpFrms[i].m_pu8Img = (const uint8_t *)m_grBuffers[i][0].m_pMem;
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
	//printf( "Nice: %d\n", nice(-5) );

	bSetAutoExposure( false );
	bSetAutoGain( false );
	bSetGain( 64 );
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

static void CvtMatToQuat(Rebels::Quaternion &q, Rebels::Point3d &p, cv::Mat &m) 
{
	assert( m.rows == 3 && m.cols == 4 );

	float grf[3][3];

	float fQ0 = 1.F;
	float fX = 0.F;
	float fY = 0.F;
	float fZ = 0.F;

	for( int i = 0; i < 3; i++ ) {
		for( int j = 0; j < 3; j++ ) {
			grf[i][j] = m.at<float>(i,j);
		}
	}
	float fT = grf[0][0] + grf[1][1] + grf[2][2] + 1;
	if (fT > 0.00000001F) {
		float fS = sqrt(fT) * 2.F;
		fQ0 = 0.25F * fS;
		fX = (grf[2][1] - grf[1][2]) / fS;
		fY = (grf[0][2] - grf[2][0]) / fS;
		fZ = (grf[1][0] - grf[0][1]) / fS;
	} else if (grf[0][0] > grf[1][1] &&
			 grf[0][0] > grf[2][2]) {
		float fS =
			sqrt(1.0F + grf[0][0] - grf[1][1] - grf[2][2]) *
			2.F;
		fX = 0.25F * fS;
		fY = (grf[1][0] + grf[0][1]) / fS;
		fZ = (grf[0][2] + grf[2][0]) / fS;
		fQ0 = (grf[2][1] - grf[1][2]) / fS;
	} else if (grf[1][1] > grf[2][2]) {
		float fS =
			sqrt(1.0F + grf[1][1] - grf[0][0] - grf[2][2]) *
			2.F;
		fX = (grf[1][0] + grf[0][1]) / fS;
		fY = 0.25F * fS;
		fZ = (grf[2][1] + grf[1][2]) / fS;
		fQ0 = (grf[0][2] - grf[2][0]) / fS;
	} else {
		float fS =
			sqrt(1.0F + grf[2][2] - grf[0][0] - grf[1][1]) *
			2.F;
		fX = (grf[0][2] + grf[2][0]) / fS;
		fY = (grf[2][1] + grf[1][2]) / fS;
		fZ = 0.25F * fS;
		fQ0 = (grf[1][0] - grf[0][1]) / fS;
	}

	q = Rebels::Quaternion( fQ0, fX, fY, fZ );
	p = Rebels::Point3d( m.at<float>(0,3), m.at<float>(1,3), m.at<float>(2,3) );
}

void bpiCamera::procColorFrame( bpiFrame *pFrm, cv::Mat &mat )
{
	uint8_t	const *p = pFrm->pu8Image();
	int nRows = mat.rows;
	int nCols = mat.cols;
	uchar *ppix = mat.ptr(0,0);
	uint8_t ug;
	uint8_t const *p2 = p + m_nCols;
	for( int r = 0; r < nRows; r++, p += (m_nCols *1), p2 += (m_nCols * 1) )
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
//			p += 2;
//			p2 += 2;
#endif
		}
	}
}


#if 1
void bpiCamera::workerThreadFn()
{
	std::unique_lock<std::mutex> l( m_queueMutex );
	m_nWorkersRunning++;
	l.unlock();

//Focal Length: 976.804 794.763
//PP: 336.136 234.617
	AprilTagPoseEstimator::Config atPEstConfig = {
		.tagSize = units::length::inch_t(6.0),
		.fx = m_cp.pinhole( 0, 0 ),
		.fy = m_cp.pinhole( 1, 1 ),
		.cx = m_cp.pinhole( 0, 2 ),
		.cy = m_cp.pinhole( 1, 2 ),
	};
	AprilTagPoseEstimator atPEst	= AprilTagPoseEstimator( atPEstConfig );

	wpi::SmallVector<double, 24> svHdgs;
	for( int i = 0; i < 24; i++ )
		svHdgs.push_back(-1.F);

	cv::Mat color( m_nRows/2, m_nCols/2, CV_8UC3 );
	while( !m_bShutdown )
	{
		bpiFrame *pFrm = waitForWork();

//fputc( '.', stdout ); fflush( stdout );
		uint32_t u32Frm = pFrm->u32Sequence() % m_nExposures;

		cv::Mat raw(m_nRows, m_nCols, CV_8UC1, (void *)pFrm->pu8Image() );
//		cv::Mat raw(m_nActiveRows, m_nCols, CV_8UC1, (void *)(pFrm->pu8Image() + m_nSkipRows * m_nCols) );
		if (m_mtxAprilTag.try_lock()) {
#if 1
			m_disp.Display( raw, 0, 0 );
#if OPTIMIZE_WINDOW
			// only look in window around previous tag
			AprilTagDetector::Results detections = m_atd.Detect(pFrm->nCols(), m_nActiveRows, (uint8_t *)pFrm->pu8Image() + m_nSkipRows * m_nCols );
#else
			// Use full image for april tags
			AprilTagDetector::Results detections = m_atd.Detect(pFrm->nCols(), pFrm->nRows(), (uint8_t *)pFrm->pu8Image() );
#endif
			int nDL = 8;

	
			for( int i = 0; i< 24; i++ ) {
				svHdgs[i] = -1.F;
			}

			float fBestDist = 1.0e10f;
			int nBestTag = -1;
			Transform3d x3dBest;
			int nMinRow = 480;
			int nMaxRow = 0;
			for (AprilTagDetection const *detection : detections ) {
				// run pose estimator
				//Transform3d pose;
				int nId = detection->GetId();
				if (nId >= 1 && nId <= 8) {
#if 0
					double grdUOrg[8];
					double grdU[8];
					
					for( int c = 0; c < 4; c++ ) {
						auto p = detection->GetCorner( c );
						
#if OPTIMIZE_WINDOW
						p.y += (float)m_nSkipRows;
#endif
						if( (int)p.y < nMinRow )
							nMinRow = (int)p.y;
						if( (int)p.y > nMaxRow )
							nMaxRow = (int)(p.y);
						float fU = p.x;
						float fV = p.y;

						float fUU; //Undistorted U
						float fUV; //Undistorted V
						grdUOrg[c * 2]     = p.x;
						grdUOrg[c * 2 + 1] = p.y;
						m_cp.UndistortPixels( fU, fV, fUU, fUV );
//						printf( "%8.4f  %8.4f -> %8.4f  %8.4f       ", fU, fV, fUU, fUV );
						grdU[c * 2]     = fUU;
						grdU[c * 2 + 1] = fUV;

						cv::Point cp1( p.x, p.y );
						
						auto pn = detection->GetCorner( (c + 1) & 3); 
#if OPTIMIZE_WINDOW
						pn.y += (float)m_nSkipRows;
#endif
						cv::Point cp2( pn.x, pn.y );

						cv::line( raw, cp1, cp2, cv::Scalar( 255 ), 2 );

					}
//					printf( "\n" );

					AprilTagDetection *pDet = (AprilTagDetection *)detection;
#if USE_DISTORTION
					pDet->SetCorners( grdU );
#else
//					pDet->SetCorners( grdUOrg );
#endif
					Transform3d x3dTag = atPEst.Estimate(*detection);

#else
					for( int c = 0; c < 4; c++ ) {
						auto p = detection->GetCorner( c );
printf( "%8.2f %8.f\n", p.x, p.y );
					}
					Transform3d x3dTag = atPEst.Estimate(*detection);
#endif
					Translation3d x3dRobot( -7.5_in, 0_in, -6.5_in );

					x3dTag = Transform3d( x3dTag.Translation() + x3dRobot, x3dTag.Rotation() );
					float fX = x3dTag.X().value();
					float fZ = x3dTag.Z().value();

					float fDistSqd = fX*fX + fZ*fZ;
					if( nId != 4 && nId != 5 && fDistSqd < fBestDist ) {
						nBestTag = nId;
						fBestDist = fDistSqd;
						x3dBest = x3dTag;
					}
//					svHdgs[(nId-1) * 3    ] = x3dTag.X().value() - 7.5 * 0.0254;
//					svHdgs[(nId-1) * 3 + 1] = x3dTag.Z().value() - 6.5 * 0.0254;
					svHdgs[(nId-1) * 3    ] = x3dTag.X().value();
					svHdgs[(nId-1) * 3 + 1] = x3dTag.Z().value();
					svHdgs[(nId-1) * 3 + 2] = x3dTag.Rotation().Y().value();
				}
#if INCLUDE_GUI
				mvprintw( nDL++, 1, "Have detection: %d", detection->GetId() );
				clrtoeol();
				mvprintw( nDL++, 1, "%s", detection->GetFamily().data() );
				clrtoeol();
				mvprintw( nDL, 30, "%8.2f %8.2f %8.2f", pose.X(), pose.Y(), pose.Z() );
				mvprintw( nDL + 1, 30, "%8.4f %8.4f %8.4f", pose.Rotation().X(), pose.Rotation().Y(), pose.Rotation().Z() );
				for (int c = 0; c<4; c++ ) {
					auto p = detection->GetCorner( c );
					mvprintw( nDL++, 1, "   %.2lf  %.2lf", p.x, p.y );
				}
#endif
			}

			struct timespec tsCur;
			struct timeval tvCur;
			struct timeval tvDelta;

			// Get current time since power on
			clock_gettime(CLOCK_MONOTONIC, &tsCur );
			TIMESPEC_TO_TIMEVAL( &tvCur, &tsCur );

			// Calculate delta between when frame was catured and now.
			timersub( &tvCur, pFrm->ts(), &tvDelta );

			// Get current NetTables time and subtract off the delta
			int64_t time = nt::Now();
			time -= tvDelta.tv_usec;
				
			m_dapTVs.Set( svHdgs, time );

#if 1
			if( nBestTag >= 1 ) {
				Pose3d pose;

				m_nSkipRows = (nMinRow - skipMargin) & 0xFFFFFFFE;
				if( m_nSkipRows < 0 )
					m_nSkipRows = 0;
				m_nActiveRows = (nMaxRow - m_nSkipRows + skipMargin) & 0xFFFFFFFE;
				if( m_nActiveRows + m_nSkipRows > 480 )
					m_nActiveRows = 480 - m_nSkipRows;
						
				if( fBestDist < 9.0f ) {
//					Transform3d x( Translation3d(x3dBest.X(), x3dBest.Y(), x3dBest.Z()), x3dBest.Rotation() );
					pose = m_p3d.TransformBy(myField[nBestTag] + x3dBest.Inverse() + m_x3CamToRobot);
					Pose2d p2d = pose.ToPose2d();
					float fX = raX.fAddValue( p2d.X().value() );
					float fY = raY.fAddValue( p2d.Y().value() );
					float fR = (float)p2d.Rotation().Radians().value();
					if( fR < 0. )
						fR += 2 * PI;
					fR = raR.fAddValue( fR );

					if( raX.fRange() > 0.2f || raY.fRange() > 0.2f ) {
						m_dapXYR.Set( wpi::SmallVector<double, 9> { -1.f, -1.f, -1.f }, time );
					}
					else {
						m_dapXYR.Set( wpi::SmallVector<double, 9> { p2d.X().value(), p2d.Y().value(), p2d.Rotation().Radians().value() }, time );

						m_p2dLast = p2d;
//						printf( "X: %8.4f/%8.4f Y: %8.4F/%8.4f rows:%d/%d %d/%d\n", fX, raX.fRange(), fY, raY.fRange(), nMinRow, nMaxRow, m_nSkipRows, m_nActiveRows );
//						printf( "X: %8.4f/%8.4f Y: %8.4F/%8.4f R: %8.6f/%8.6f rows:%d/%d %d/%d\n", p2d.X().value(), raX.fRange(), p2d.Y().value(), raY.fRange(), p2d.Rotation().Radians().value(), raR.fRange(), nMinRow, nMaxRow, m_nSkipRows, m_nActiveRows );
					}
				}
				else {
					m_dapXYR.Set( wpi::SmallVector<double, 9> { -1.f, -1.f, -1.f }, time );
//					m_dapXYR.Set( wpi::SmallVector<double, 9> { m_p2dLast.X().value(), m_p2dLast.Y().value(), m_p2dLast.Rotation().Radians().value() }, time );
					m_p2dLast = Pose2d( -1_m, -1_m, -1_rad );
				}
			}
			else {
				int64_t time = nt::Now();
				m_dapXYR.Set( wpi::SmallVector<double, 9> { -1.f, -1.f, -1.f }, time );
				m_nSkipRows = 0;
				m_nActiveRows = 480;
			}
#endif
#if INCLUDE_GUI
			refresh();
#endif

#endif
			m_mtxAprilTag.unlock();
			// Get current time since power on
			clock_gettime(CLOCK_MONOTONIC, &tsCur );

			// Calculate delta between when frame was catured and now.
			timersub( &tvCur, pFrm->ts(), &tvDelta );

			char szTmp[64];
			sprintf( szTmp, "%d.%06d", tvDelta.tv_sec, tvDelta.tv_usec );
			cv::putText( raw, szTmp, cv::Point( 10, 30 ), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255), 2);

			cv::line( raw, cv::Point(1,m_nSkipRows), cv::Point(m_nCols-3, m_nSkipRows), cv::Scalar(255), 2 );
			cv::line( raw, cv::Point(1,m_nSkipRows+m_nActiveRows), cv::Point(m_nCols-3, m_nSkipRows+m_nActiveRows), cv::Scalar(255), 2 );
			m_outputStream.PutFrame(raw);

		}
		if (m_mtxVideo.try_lock()) {
#if 0
#if 0
			procColorFrame(pFrm, color );
			m_outputStream.PutFrame(color);
#else
			
			m_outputStream.PutFrame(raw);
#endif
#endif
#if defined(LOG_DIR)
			if (!m_bLoggingDone ) {
				if( m_fp ) {
					m_bpLogging.Set( true );
#if 0
					fprintf( m_fp, "<<FRM>>\n%d\n%d\n", pFrm->u32Sequence(), pFrm->u32Exposure() );
					fprintf( m_fp, "%d\n%d\n", pFrm->nRows(), pFrm->nCols() );
					if (fwrite( pFrm->pu8Image(), pFrm->nCols(), pFrm->nRows(), m_fp ) != pFrm->nRows()) {
						fclose( m_fp );
						m_fp = 0;
					}
#else
					cv::Mat raw1(m_nRows, m_nCols, CV_8UC1, (void *)pFrm->pu8Image() );
			        vector<uchar> vPNG;
			        cv::imencode( ".png", raw1, vPNG );

					fprintf( m_fp, "<<FRMPNG>>\n%d\n%d\n", pFrm->u32Sequence(), pFrm->u32Exposure() );
					fprintf( m_fp, "%d\n", vPNG.size() );
					if (fwrite( (const char*)vPNG.data(), 1, vPNG.size(), m_fp ) != vPNG.size()) {
						fclose( m_fp );
						m_fp = 0;
					}
					else {
						fdatasync( fileno(m_fp) );
					}
#endif
					if( time(0) > m_tmLogEnd ) {
						m_bLoggingDone = true;
						fflush( m_fp );
						fclose( m_fp );
						m_bpLogging.Set( false );
						m_fp = 0;
					}
				}
				else {
					auto bStart = m_bsIsLogging.Get();
					if( bStart ) {
						char szTmp[128];
						for (int i = 0; ; i++ )
						{
							snprintf( szTmp, sizeof(szTmp), "%s/cam%04d.log", LOG_DIR, i );
							if (access( szTmp, F_OK ) == -1) 
								break;
						}
						printf( "Opening log file at %s\n", szTmp );
						m_fp = fopen( szTmp, "w" );
						m_tmLogEnd = time(0) + 200;
					}
				}
			}
			else {
			}

#endif
			m_mtxVideo.unlock();
		}
#if defined(LOG_DIR1)
		else {
			printf( "m_bLoggingDone = %d\n", m_bLoggingDone );
			if (!m_bLoggingDone && m_mtxFileWriter.try_lock()) {
				if( m_fp ) {
					fprintf( m_fp, "<<FRM>>\n%d\n%d\n", pFrm->u32Sequence(), pFrm->u32Exposure() );
					if (fwrite( pFrm->pu8Image(), pFrm->nCols(), pFrm->nRows(), m_fp ) != pFrm->nRows()) {
						fclose( m_fp );
						m_fp = 0;
					}
					if( time(0) > m_tmLogEnd ) {
						m_bLoggingDone = true;
						fclose( m_fp );
						m_fp = 0;
					}
				}
				else {
					auto bStart = m_bsIsLogging.Get();
					printf( "Logging = %d\n", bStart);

					if( bStart ) {
						char szTmp[128];
						for (int i = 0; ; i++ )
						{
							snprintf( szTmp, sizeof(szTmp), "%s/cam%04d.log", LOG_DIR, i );
							if (access( szTmp, F_OK ) == -1) 
								break;
						}
						printf( "Opening log file at %s\n", szTmp );
						m_fp = fopen( szTmp, "w" );
						m_tmLogEnd = time(0) + 160;
					}
					
				}
				m_mtxFileWriter.unlock();
			}
		}
#endif

		if( m_bSaveNext && m_nSaveType == 0 ) {
			m_bSaveNext = false;
			cv::imwrite( m_strFile, raw );
		}
#if 0		
		if (!u32Frm) {
			procColorFrame( pFrm, color );
			m_outputStream.PutFrame(color);
		}
#endif
		FrameComplete( pFrm );
	}
	l.lock();
	printf( "Worker %d stopping\n", m_nWorkersRunning );
	m_nWorkersRunning--;
	l.unlock();
}
#endif

void bpiCamera::vSaveImage( char const *psz, int nType ) //nType: 0 = raw Bayer, 1 = rgb, 2 = green
{
	m_bSaveNext = true;
	m_strFile = psz;
	m_nSaveType = nType;
}
