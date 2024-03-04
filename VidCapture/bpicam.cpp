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
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <string>
#include <stdexcept>

#include <sys/time.h>
#include <sys/resource.h>
#include <sched.h>

using namespace std;

#include "v4lcam-mp.h"
#include "bpicam.h"

#define DESIRED_GREEN_INTENSITY	200

bpiCamera::bpiCamera( char const *pszVid, int nExposures, int nCam, char const *pszLog ) :
	v4lcam( pszVid ),
	m_nExposures( nExposures ),
	m_grpFrms( 0 ),
	m_bShutdown( false ),
	m_nCam( nCam ),
	m_nWorkersRunning( 0 ),
	m_nWorkersWaiting( 0 ),
	m_pNextFrame( 0 ),
	m_vl( pszLog, true )
{
	if( m_nExposures > MAX_EXPOSURES )
		throw invalid_argument( string( "bpiCamera(): num exposures exceeds max" ) );

//	printf( "Instance:: %x\n", (uint32_t)frc::CameraServer::GetInstance() );

	//annotatedOutput = frc::CameraServer::GetInstance()->PutVideo( "test", 160, 120 );
	bSetSelection( 1, 5, 640, 480 );
	bSetRes( 640, 480, V4L2_PIX_FMT_SGRBG8 );

	m_u32MaxExp = 480;
	m_fdLog = open( "cam.log", O_CREAT | O_RDWR, 0660 );
	for( int i = 0; i < nExposures; i++ )
	{
		m_gruNextExp[i] = 480;
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
	if( m_fdLog >= 0 )
		close( m_fdLog );
	
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
	printf( "Nice: %d\n", nice(-5) );

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

void bpiCamera::procColorFrame( bpiFrame *pFrm, cv::Mat &mat )
{
	uint8_t	const *p = pFrm->pu8Image();
	int nRows = mat.rows;
	int nCols = mat.cols;
	uchar *ppix = mat.ptr(0,0);
	uint8_t ug;
	uint8_t const *p2 = p + m_nCols;
	for( int r = 0; r < nRows; r++, p += (m_nCols *3), p2 += (m_nCols * 3) )
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

void bpiCamera::workerThreadFn()
{
	int nRows = m_nRows/2;
	int nCols = m_nCols/2;
	cv::Mat green( nRows, nCols, CV_8UC1 );
	cv::Mat color( nRows/2, nCols/2, CV_8UC3 );
	cv::Mat blurOut;
	cv::Mat ThreshOut;

	std::unique_lock<std::mutex> l( m_queueMutex );
	m_nWorkersRunning++;
	printf( "Worker %d started\n", m_nWorkersRunning );
	printf( "sch: %d\n", sched_getscheduler(0) );
	printf( "pri: %d\n", getpriority(PRIO_PROCESS, 0) );
	l.unlock();
	while( !m_bShutdown )
	{
		bpiFrame *pFrm = waitForWork();

//fputc( '.', stdout ); fflush( stdout );
		uint32_t u32Frm = pFrm->u32Sequence() % m_nExposures;

		
		cv::Mat raw( m_nRows, m_nCols, CV_8UC1, (void *)pFrm->pu8Image() );

		m_vl.bWriteRaw( raw, pFrm->u32Sequence(), pFrm->u32Exposure() );
//		m_vl.bWritePNG( raw, pFrm->u32Sequence(), pFrm->u32Exposure() );
		printf( "%d\n", pFrm->u32Sequence() );

		if( u32Frm )
		{
			uint8_t u8Max;
			procGreenFrame( pFrm, green, u8Max );
			uint32_t u32Exp;
			if( u8Max > 250 )
				u32Exp = pFrm->u32Exposure() / 2;
			else
			{
				u32Exp = (uint32_t)((float)pFrm->u32Exposure() * DESIRED_GREEN_INTENSITY / u8Max);
			}
			UpdateExposure( pFrm->u32Sequence(), u32Exp );
		}
		else
		{
			procColorFrame( pFrm, color );
		}
		FrameComplete( pFrm );
	}
	l.lock();
	printf( "Worker %d stopping\n", m_nWorkersRunning );
	m_nWorkersRunning--;
	l.unlock();
}

