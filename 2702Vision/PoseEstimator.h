
class RobotPoseEstimator
{
	Rebels::CamParm const			&m_cp;
	std::vector<cv::Point2f>		m_vImgCoords;
	std::vector<cv::Point3f> const	&m_vPointCoords;

	frc::Transform2d				m_x2d;

	static int Evaluate( void *p, int m, int n, const float *pdfX, float *pdfVals, int nFlg );

	float Evaluate( int m, int n, const float *pdfX, float *pdfVals, int nFlg );
public:
	RobotPoseEstimator(Rebels::CamParm const &cp, std::vector<cv::Point2f> const &vImgCoords, std::vector<cv::Point3f> const &vPointCoords );
	~RobotPoseEstimator();
	
	frc::Transform2d EstimatePose2D();
};

