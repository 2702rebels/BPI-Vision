#ifndef BPICAM_H
#define BPICAM_H

// maximum # of managed exposure times.
#define MAX_EXPOSURES	8
#define NUM_WORKERS	4

#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>

class bpiFrame
{
	friend class bpiCamera;
	int				m_nBuffer;
	int				m_nRows;
	int				m_nCols;
	uint32_t		m_u32Seq;
	uint32_t		m_u32Exp;
	uint8_t	const	*m_pu8Img;
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
	inline const struct timeval *ts() const
	{
		return &m_ts;
	}
};

class bpiCamera : public v4lcam
{
	int			m_fdLog;
	int			m_nExposures;
	uint32_t	m_u32MaxExp;
	uint32_t	m_gruCurExp[MAX_EXPOSURES];
	uint32_t	m_gruNextExp[MAX_EXPOSURES];
	bool 		m_bSave = true;
	bool			m_bSaveNext = false;
	std::string		m_strFile;
	int				m_nSaveType = 0;
	bool			m_bApril = false;

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

	fbDisp			m_disp;
	cs::CvSource annotatedOutput;
	frc::AprilTagDetector		m_atd;

public:
	bpiCamera( char const *pszVid, int nExposures, int nCam );
	virtual ~bpiCamera();
	virtual void UpdateExposure( uint32_t u32Frame, uint32_t u32Exposure );
	virtual bool bProcessFrame( v4l2_buffer &buf );
	virtual bool bStart();
	virtual bool bStop();
	virtual void vSaveImage( char const *psz, int nType ); //nType: 0 = raw Bayer, 1 = rgb, 2 = green

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


#endif
