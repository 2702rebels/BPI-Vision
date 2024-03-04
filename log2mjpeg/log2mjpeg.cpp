#include <fstream>
#include <vector>

#include <opencv2/imgcodecs.hpp>

using namespace std;

#include "VisLog.h"
#include "Bayer2RGB.h"
#include "Bayer2Grey.h"

enum pmode { pmcolor, pmcolor2, pmgrey, pmgrey2, pmbayer, pmgreen, pmred, pmblue };

pmode procmode = pmcolor;

void Bayer2Red( cv::Mat &matRed, cv::Mat const &matBayer )
{
	assert( matRed.rows == matBayer.rows / 2 && matRed.cols == matBayer.cols / 2 );
	assert( matRed.type() == CV_8UC1 && matBayer.type() == CV_8UC1 );
	assert( (matBayer.rows & 1) == 0 && (matBayer.cols & 1) == 0 );

	for( int r = 0; r < matRed.rows; r++ ) {
		uint8_t const *pu8Bayer = matBayer.ptr( r * 2 ) + 1;
		uint8_t *pu8Red = matRed.ptr( r );
		for( int c = 0; c < matRed.cols; c++ ) {
			*pu8Red++ = *pu8Bayer;
			pu8Bayer += 2;
		}
	}
}

void Bayer2Blue( cv::Mat &matBlue, cv::Mat const &matBayer )
{
	assert( matBlue.rows == matBayer.rows / 2 && matBlue.cols == matBayer.cols / 2 );
	assert( matBlue.type() == CV_8UC1 && matBayer.type() == CV_8UC1 );
	assert( (matBayer.rows & 1) == 0 && (matBayer.cols & 1) == 0 );

	for( int r = 0; r < matBlue.rows; r++ ) {
		uint8_t const *pu8Bayer = matBayer.ptr( r * 2 + 1 );
		uint8_t *pu8Blue = matBlue.ptr( r );
		for( int c = 0; c < matBlue.cols; c++ ) {
			*pu8Blue++ = *pu8Bayer;
			pu8Bayer += 2;
		}
	}
}

void Bayer2Green( cv::Mat &matGreen, cv::Mat const &matBayer )
{
	assert( matGreen.rows == matBayer.rows / 2 && matGreen.cols == matBayer.cols / 2 );
	assert( matGreen.type() == CV_8UC1 && matBayer.type() == CV_8UC1 );
	assert( (matBayer.rows & 1) == 0 && (matBayer.cols & 1) == 0 );

	int nColsPlus1 = matBayer.cols + 1; 
	for( int r = 0; r < matGreen.rows; r++ ) {
		uint8_t const *pu8Bayer = matBayer.ptr( r * 2 );
		uint8_t *pu8Green = matGreen.ptr( r );
		for( int c = 0; c < matGreen.cols; c++ ) {
			*pu8Green++ = (pu8Bayer[0] + pu8Bayer[nColsPlus1]) /2;
			pu8Bayer += 2;
		}
	}
}

void Usage()
{
	cerr << "Usage: log2mjpeg [-colour|colour2|grey|grey2|bayer|red|green|blue] [-iimgfilespec] infile outfile\n";
	cerr << "      colour     -> RGB\n";
	cerr << "      colour2    -> RGB 1/2 resolution\n";
	cerr << "      grey       -> GreyScale\n";
	cerr << "      grey2      -> GreyScale 1/2 resolution\n";
	cerr << "      beyer      -> Raw Bayer frame\n";
	cerr << "      red        -> Extracts the red pixels\n";
	cerr << "      green      -> Extracts the green pixels\n";
	cerr << "      blue       -> Extracts the blue pixels\n";
	cerr << "     -iimgfilespec  saves each image to a file ... eg: img%04d.png\n";

	exit(1);
}

int main( int argc, char **argv )
{
	char const *pszIn = 0;
	char const *pszOut = 0;
	char const *pszImgFileSpec = 0;
	for( int i = 1; i < argc; i++ ) {
		if( argv[i][0] == '-' ) {
			if( !strcmp( argv[i], "-colour" ) )
				procmode = pmcolor;
			else if( !strcmp( argv[i], "-colour2" ) )
				procmode = pmcolor2;
			else if( !strcmp( argv[i], "-grey" ) )
				procmode = pmgrey;
			else if( !strcmp( argv[i], "-grey2" ) )
				procmode = pmgrey2;
			else if( !strcmp( argv[i], "-bayer" ) )
				procmode = pmbayer;
			else if( !strcmp( argv[i], "-red" ) )
				procmode = pmred;
			else if( !strcmp( argv[i], "-green" ) )
				procmode = pmgreen;
			else if( !strcmp( argv[i], "-blue" ) )
				procmode = pmblue;
			else if( argv[i][1] == 'i' )
				pszImgFileSpec = argv[i] + 2;
			else
				Usage();
		}
		else if( !pszIn )
			pszIn = argv[i];
		else if( !pszOut )
			pszOut = argv[i];
		else
			Usage();
	}
	if( !pszOut ) {
		Usage();
		return 1;
	}

	VisionLog vl( pszIn );
	ofstream of( pszOut );
	if( !of.is_open() ) {
		cerr << "Unable to open output file\n";
		return 1;
	}

	cv::Mat img;
	int nExp;
	int nSeq;
	while( (nSeq = vl.i32ReadNext( img, nExp )) >= 0 ) {
		cout << nSeq << endl;

		cv::Mat imgOut;
		switch( procmode ) {
		case pmgrey:
			{
				cv::Mat matGrey( img.rows, img.cols, CV_8UC1 );
				Bayer2Grey( matGrey, img );
				imgOut = matGrey;
			}
			break;
		case pmgrey2:
			{
				cv::Mat matGrey( img.rows/2, img.cols/2, CV_8UC1 );
				Bayer2HalfGrey( matGrey, img );
				imgOut = matGrey;
			}
			break;
		case pmbayer:
			imgOut = img;
			break;
		case pmred:
			{
				cv::Mat matRed( img.rows/2, img.cols/2, CV_8UC1 );
				Bayer2Red( matRed, img );
				imgOut = matRed;
			}
			break;
		case pmgreen:
			{
				cv::Mat matGreen( img.rows/2, img.cols/2, CV_8UC1 );
				Bayer2Green( matGreen, img );
				imgOut = matGreen;
			}
			break;
		case pmblue:
			{
				cv::Mat matBlue( img.rows/2, img.cols/2, CV_8UC1 );
				Bayer2Blue( matBlue, img );
				imgOut = matBlue;
			}
			break;
		case pmcolor2:
			{
				cv::Mat matRGB( img.rows/2, img.cols/2, CV_8UC3 );
				Bayer2HalfRGB( matRGB, img );
				imgOut = matRGB;
			}
			break;
		case pmcolor:
		default:
			{
				cv::Mat matRGB( img.rows, img.cols, CV_8UC3 );
				Bayer2RGB( matRGB, img );
				imgOut = matRGB;
			}
			break;
		}

		if( pszImgFileSpec ) {
			char szTmp[1024];
			sprintf( szTmp, pszImgFileSpec, nSeq );
			cv::imwrite( szTmp, imgOut ); 
		}
		vector<uchar> vJPEG;
		cv::imencode( ".jpg", imgOut, vJPEG );

		of.write( (const char *)vJPEG.data(), vJPEG.size() );
		
	}
	return 0;
}
