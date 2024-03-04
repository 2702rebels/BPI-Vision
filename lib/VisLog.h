#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>


class VisionLog
{
	fstream m_fs;
	bool 	m_bWrite;

	std::mutex	m_mtx;

	int32_t i32ReadVal();

public:
	VisionLog( char const *pszFile, bool bWrite = false );
	~VisionLog();

	int32_t i32ReadNext( cv::Mat &mat, int &nExp );
	bool bWriteRaw( cv::Mat const &img, uint32_t u32Seq, uint32_t u32Exp );
	bool bWritePNG( cv::Mat const &img, uint32_t u32Seq, uint32_t u32Exp );
};
