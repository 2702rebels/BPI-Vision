
class FieldXfrms {
	frc::Transform3d	m_grxfmTags[8];

public:
	FieldXfrms();
	~FieldXfrms();

	// get transform for Tag ID
	frc::Transform3d const &operator[]( int nId ) const {
		if (nId < 1 || nId > 8)
			throw runtime_error( string( "FieldXfrm: Tag ID out of range") );

		return m_grxfmTags[nId - 1];
	}

	void GetTagCorners( int nTag, std::vector<cv::Point3f> &vPts );
};
