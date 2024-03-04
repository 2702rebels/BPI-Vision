#include <string>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <stdarg.h>

using namespace std;

#include <opencv2/imgcodecs.hpp>
#include "VisLog.h"


VisionLog::VisionLog( char const *psz, bool bWrite ) 
{
	m_bWrite = bWrite;
	m_fs.open( psz, bWrite ? fstream::out : fstream::in );
	if( !m_fs.is_open() )
		throw std::runtime_error( string( "Unable to open log file" ) );
}

VisionLog::~VisionLog() 
{
}

int32_t VisionLog::i32ReadVal()
{
	string str;
	if( !getline( m_fs, str ) )
		throw runtime_error( string( "VisionLog::u32ReadVal() - unexpected end of file?" ) );

	return atoi( str.c_str() );
}

// returns sequence #
int32_t VisionLog::i32ReadNext( cv::Mat &img, int &nExp )
{
	if( m_bWrite )
		throw runtime_error( "VisionLog::bReadNext() - file is open for writing" );

	if( !m_fs.is_open() )
		throw runtime_error( string( "VisonLog::bReadNext() - file is not open" ) );

	std::unique_lock<std::mutex> l( m_mtx );

	string str;
	if( !getline( m_fs, str ) )
		return -1;

	if( !str.compare( "<<FRMPNG>>" )) {
		int nSeq = i32ReadVal();
		nExp = i32ReadVal();
		int nSize = i32ReadVal();

		uint8_t gru8Buff[nSize];
		if( !m_fs.read( (char *)gru8Buff, nSize ) )
			throw runtime_error( string( "VisionLog::bReadNext() - failed reading frame" ) );

		cv::Mat matImg;
		matImg = cv::imdecode(cv::Mat(1, nSize, CV_8UC1, gru8Buff), cv::IMREAD_UNCHANGED);
		img = matImg;

		return nSeq;
	}
	else if( !str.compare( "<<FRM>>" ) ) {
		int nSeq = i32ReadVal();
		nExp = i32ReadVal();
		int nRows = i32ReadVal();
		int nCols = i32ReadVal();

		cv::Mat matIn( nRows, nCols, CV_8UC1 );

		if( !m_fs.read( (char *)matIn.ptr(), nRows * nCols ) ) 
			throw runtime_error( string( "VisionLog::bReadNext() - failed reading frame" ) );

		img = matIn;
		return nSeq;
	}
	else {
		if( m_fs.eof() )
			return -1;
		throw runtime_error( string( "VisionLog::bReadNext() - <<FRM>> not found" ) );
	}

	return -1;
}

static std::string strPrintf( char const *psz, ... )
{
	va_list va;

	va_start( va, psz );
	int len = vsnprintf( 0, 0, psz, va );
	va_end( va );

	char szTmp[len + 1];
				
	va_start( va, psz );
	len = vsnprintf( szTmp, len + 1, psz, va );
	va_end( va );
	return std::string(szTmp);
}


bool VisionLog::bWriteRaw( cv::Mat const &img, uint32_t u32Seq, uint32_t u32Exp )
{
	std::unique_lock<std::mutex> l( m_mtx );
	m_fs << strPrintf( "<<FRM>>\n%u\n%u\n", u32Seq, u32Exp );
	m_fs << strPrintf( "%d\n%d\n", img.rows, img.cols );
	m_fs.write( (char *)img.data, img.rows * img.cols );

	return m_fs.failbit ? false : true;
}

bool VisionLog::bWritePNG( cv::Mat const &img, uint32_t u32Seq, uint32_t u32Exp )
{
	std::unique_lock<std::mutex> l( m_mtx );
	vector<uchar> vPNG;
	cv::imencode( ".png", img, vPNG );

	m_fs << strPrintf( "<<FRMPNG>>\n%u\n%u\n", u32Seq, u32Exp );
	m_fs << strPrintf( "%d\n", vPNG.size() );
	m_fs.write( (char *)vPNG.data(), vPNG.size() );

	return m_fs.failbit ? false : true;
}
