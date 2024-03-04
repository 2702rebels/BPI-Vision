class MonoCalib
{
	CamParm &m_Cam;
	vector<string> m_log;
	vector<string> m_res;

	vector<vector<Point2d> > &m_calPoints;
	
	float m_fBarLength;
	float m_fFloorHeight;

	static int Evaluate( void *p, int m, int n, const float *pdfX, float *pdfVals, int nFlg );
	
	void UpdateCamXfrms( const float *pfX );
	int Evaluate( int m, int n, const float *pfX, float *pfY, int nFlg );

public:
	MonoCalib( CamParm &cam, vector<vector<Point2d> > &calPts );
	~MonoCalib();

	float fCalibrate( float fBarLength, float fFloorHeight );
	
	void saveLog( char const *psz );
	vector<string> const& results() const { return m_res; }
};
