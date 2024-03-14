#pragma once

#include "v4lcam-mp.h"

#if DISPLAY
#include "fbdisp.h"
#endif
#if CAMSERVER
#include "cameraserver/CameraServer.h"
#endif

#include "VisLog.h"

// maximum # of managed exposure times.
#define MAX_EXPOSURES	8
#define NUM_WORKERS	4

class bpiFrame
{
	friend class bpiCamera;
	int				m_nBuffer;
	int				m_nRows;
	int				m_nCols;
	uint32_t		m_u32Seq;
	uint32_t		m_u32Exp;
	uint8_t			*m_pu8Img;
	struct timeval	m_ts;
public:
	bpiFrame() :
		m_pu8Img( 0 )
	{
	}
	~bpiFrame()
	{
	}
	inline uint32_t u32Sequence() const 
	{
		return m_u32Seq;
	}
	inline uint32_t u32Exposure() const
	{
		return m_u32Exp;
	}
	inline int nRows() const
	{
		return m_nRows;
	}
	inline int nCols() const
	{
		return m_nCols;
	}
	inline uint8_t const *pu8Image() const
	{
		return m_pu8Img;
	}
	inline uint8_t *pu8Image()
	{
		return m_pu8Img;
	}
	inline const struct timeval *ts() const
	{
		return &m_ts;
	}
};

class bpiCamera : public v4lcam
{
	int			m_nExposures;
	uint32_t	m_u32MaxExp;
	uint32_t	m_gruCurExp[MAX_EXPOSURES];
	uint32_t	m_gruNextExp[MAX_EXPOSURES];
	bool 		m_bSave = true;

	bpiFrame	*m_grpFrms;

	std::thread	m_grthWorkers[NUM_WORKERS];
	std::thread m_thCamera;

	std::mutex	m_queueMutex;
	std::condition_variable	m_queueCV;;

	bool		m_bShutdown;
	int			m_nCam;
	int			m_nWorkersRunning;
	int			m_nWorkersWaiting;
	bpiFrame	*m_pNextFrame;

#if DISPLAY
	fbDisp			m_disp;
#endif
#if CAMSERVER
	cs::CvSource	m_cs;
	std::mutex		m_mtxCS;
#endif
	cs::CvSource annotatedOutput;

	std::mutex		m_mtxLog;
	VisionLog		m_vl;

	bool			m_bLoggingDone = false;
	time_t			m_tmLogEnd;

	nt::DoubleArrayPublisher		m_dapXYZ;
	nt::DoubleArrayPublisher		m_dapTV;
	nt::BooleanPublisher			m_bpLogging;
	nt::BooleanSubscriber			m_bsIsLogging;
	nt::DoubleSubscriber			m_dsExposure;
public:
	bpiCamera( char const *pszVid, int nExposures, int nCam );
	virtual ~bpiCamera();
	virtual void UpdateExposure( uint32_t u32Frame, uint32_t u32Exposure );
	virtual bool bProcessFrame( v4l2_buffer &buf );
	virtual bool bStart();
	virtual bool bStop();

	virtual void FrameComplete( bpiFrame *pFrame );

	void SetThreadPriority( int policy, int pri );

	//camera thread functions
	void cameraThreadFn();
	virtual bool QueueWork( v4l2_buffer const &Buff );


	// Worker thread functions
	bpiFrame *waitForWork();
	void workerThreadFn();
	void procGreenFrame( bpiFrame *pFrm, cv::Mat &mat, uint8_t &u8Max );
	void procColorFrame( bpiFrame *pFrm, cv::Mat &mat );
};

