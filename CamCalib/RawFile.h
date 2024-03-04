
class  RawFile
{
	std::vector<std::vector<std::vector<Rebels::Point2d>>>	m_vFrames;

public:
	RawFile()
	{
	}
	~RawFile()
	{
	}
	bool bLoad( char const *psz ) {
		ifstream inf( psz );
		if( !inf.is_open() )
			return false;

		string str;
		while (getline(inf, str)) {
			int nRead = 0;
			char const *psz = str.c_str();

			int nTag;

			if( sscanf( psz, "Tag %d%n", &nTag, &nRead ) != 1 ) {
				fprintf( stderr, "Failed reading Tag from %s\n", psz );
				return false;
			}

			std::vector<std::vector<Rebels::Point2d>> vFrame;	
			// 4 corners for an AprilTag for 1st camera
			for( int i = 0; i < 4; i++ ) {
				float fU, fV;
				psz += nRead;
				if( sscanf( psz, " (%f,%f)%n", &fU, &fV, &nRead ) != 2 ){
					fprintf( stderr, "Failed reading UV %d for camera 1\n", i + i );
					return false;
				}
				vFrame.push_back( std::vector<Rebels::Point2d>{Rebels::Point2d(fU, fV)} );
			}
			// 4 corners for an AprilTag for 2nd camera
			for( int i = 0; i < 4; i++ ) {
				float fU, fV;
				psz += nRead;
				if( sscanf( psz, " ( %f, %f)%n", &fU, &fV, &nRead ) != 2 ) {
					fprintf( stderr, "Failed reading UV %d for camera 1 \'%s'\n", i + i, psz );
					return false;
				}
				vFrame[i].push_back(Rebels::Point2d{fU, fV});
			}
			m_vFrames.push_back(vFrame);	
		}
		return true;
	}

	inline size_t nFrames() const {
		return m_vFrames.size();
	}
	inline std::vector<std::vector<std::vector<Rebels::Point2d>>> const &Frames() const
	{
		return m_vFrames;
	}
	inline std::vector<std::vector<Rebels::Point2d>> &operator[](size_t n) {
		return m_vFrames[n];
	}
};
 
