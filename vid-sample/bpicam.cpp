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

#define AUTO_GAIN	0
#define APRILTAGS	0
#define ANNOTATE	0

#if 0
#define RES_ROWS	(480/2)
#define RES_COLS	(752/2)
#else
#define RES_ROWS	480
#define RES_COLS	752
#endif

#define MAX_EXP		240

#include "bpicam.h"

#include "Bayer2RGB.h"
#include "Bayer2Grey.h"

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
	m_vl( "/storage/pi/vid.log", true )
{
	if( m_nExposures > MAX_EXPOSURES )
		throw invalid_argument( string( "bpiCamera(): num exposures exceeds max" ) );

//	printf( "Instance:: %x\n", (uint32_t)frc::CameraServer::GetInstance() );

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
	m_grpFrms[nIdx].m_pu8Img = (uint8_t *)m_grBuffers[nIdx][0].m_pMem;

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

void fake( bpiFrame *pFrm )
{
	int nCols = pFrm->nCols();

	uint8_t *pu8 = pFrm->pu8Image();

	for( int r = 0; r < 16; r++, pu8 += nCols )
	{
		for( int c = 0; c < 16; c++ )
		{
			pu8[c] = r << 4 | c;
		}
	}

}

bool bFindCheckerboard( cv::Mat &mat )
{
	cv::Size patternSize( 7, 4 );
	std::vector<cv::Point2f> corners;

	bool bFound = cv::findChessboardCorners( mat, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK );

	if( bFound ) {
static int nFrms = 0;
		printf( "Have Chessboard (%05d)\n", nFrms++ );
#if 0
		for( int c = 0; c < corners.size(); c++ ) {
			auto & cp1 = corners[c];
			auto & cp2 = corners[(c + 1) % corners.size()];
			cv::line( mat, cp1, cp2, cv::Scalar( 255 ), 2 );
		}
#endif
	}
	return bFound;
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

#if APRILTAGS
	AprilTagPoseEstimator::Config atPEstConf = {
		.tagSize = units::length::inch_t(6.0),
		.fx = 478.498,
		.fy = 478.496,
		.cx = 197.701,
		.cy = 136.979
	};
	AprilTagPoseEstimator atPEst( atPEstConf );

    frc::AprilTagDetector       atd;
	atd.AddFamily("tag36h11", 0);
#endif

	while( !m_bShutdown )
	{
		bpiFrame *pFrm = waitForWork();

		uint32_t u32Frm = pFrm->u32Sequence() % m_nExposures;

		cv::Mat raw( m_nRows, m_nCols, CV_8UC1, (void *)(pFrm->pu8Image()), m_nCols );

#if APRILTAGS
		AprilTagDetector::Results detections = atd.Detect(pFrm->nCols(), pFrm->nRows(), (uint8_t *)pFrm->pu8Image() );
		for( AprilTagDetection const *det : detections ) {
			Transform3d pose = atPEst.Estimate( *det );
			printf( "Have Tag: %d\n", det->GetId() );
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
		if( m_mtxLog.try_lock() ) 
		{
static uint32_t u32NxtCheck = 0;

			if (pFrm->u32Sequence() > u32NxtCheck) {
				bool bHaveBoard = bFindCheckerboard( raw );
				if (bHaveBoard) {
					m_vl.bWriteRaw( raw, pFrm->u32Sequence(), pFrm->u32Exposure() );
					u32NxtCheck = pFrm->u32Sequence() + 120;
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

