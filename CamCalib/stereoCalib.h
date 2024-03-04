class StereoCalib
{
	StereoCam &m_Cam;
	vector<string> m_log;
	vector<string> m_res;

	cv::Mat_<cv::Vec3f> m_vRefPts;
	

	vector<vector<vector<Point2d>>> const &m_vFrames;
	
	bool m_bAlignYAxis;
	float m_fBarLength;
	float m_fFloorHeight;

	static int Evaluate( void *p, int m, int n, const float *pdfX, float *pdfVals, int nFlg );
	
	void UpdateCamXfrms( const float *pfX );
	int Evaluate( int m, int n, const float *pfX, float *pfY, int nFlg );

public:
	StereoCalib( StereoCam &cam, vector<vector<vector<Point2d>>> const &vFrames, bool bAlignYAxis = false );
	~StereoCalib();

	float fCalibrate( float fBarLength, float fFloorHeight );
	
	void saveLog( char const *psz );
	vector<string> const& results() const { return m_res; }
};
