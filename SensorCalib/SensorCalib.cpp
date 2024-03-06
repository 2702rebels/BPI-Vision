#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

#include "VisLog.h"

void Usage( char const *psz )
{
	fprintf( stderr, "Usage: %s inputfile\n", psz );
	exit(1);
}


void vAddImage( vector<vector<Point2f>> &vxImgPts, Size &sizImg, Size &sizBoard, int nWinSize, char const *pszFile )
{
	cv::Mat mat = cv::imread( pszFile, IMREAD_UNCHANGED );
	
	std::vector<cv::Point2f> corners;

	bool bFound = cv::findChessboardCorners( mat, sizBoard, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK );
	if( bFound ) {
		cv::cornerSubPix( mat, corners, Size(nWinSize,nWinSize), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001 ));
		vxImgPts.push_back(corners);
		if (sizImg.height == 0) {
			sizImg = mat.size();
		}
		else {
			if (sizImg.height != mat.rows || sizImg.width != mat.cols)
				throw runtime_error( string( pszFile ) + string(": size does not match") );
		}
	}
}

void vProcListFile( vector<vector<Point2f>> &vxImgPts, Size &sizImg, Size &sizBoard, int nWinSize, char const *pszFile )
{
	fstream fs;	
	fs.open( pszFile, fstream::in );
	if (!fs.is_open()) {
		throw runtime_error( string("Unable to open list file ") + string( pszFile ) );
	}

	char szTmp[2048];
	while( !fs.getline( szTmp, sizeof(szTmp) ).eof() ) {
		vAddImage( vxImgPts, sizImg, sizBoard, nWinSize, szTmp );
	}
	
}

static void calcBoardCornerPositions(Size const &boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.clear();

    for (int i = 0; i < boardSize.height; ++i) {
        for (int j = 0; j < boardSize.width; ++j) {
            corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
        }
    }
}

void vRandomizePoints( vector<vector<Point2f>> &vxPts, int nFrms )
{
	vector<bool> vxUsed(vxPts.size(), false);
	vector<vector<Point2f>> vxNew;

	srand(time(0));
	for (int i = 0; i < nFrms; ) {
		float fRand =  double(rand()) / RAND_MAX;

		int nRand = int(fRand * (vxPts.size() - 1) + 0.5); 
		if (!vxUsed[nRand]) {
			vxUsed[nRand] = true;
			vxNew.push_back(vxPts[nRand]);
			i++;
		}
		else {
		}
	}

	vxPts = vxNew;
}

int main( int argc, char **argv )
{
	Size sizBoard(7, 4);
	Size sizImg;
	float fSqSz = 31.4;
	int nWinSize = 11;
	char const *pszIn = 0;
	char const *pszOutPref = 0;
	char const *pszInList = 0;
	int nRandom = -1;

	vector<vector<cv::Point2f>> imagePoints;

	try {
		for( int i = 1; i < argc; i++ ) {
			if (argv[i][0] == '-') {
				switch (argv[i][1]) {
				case 'b':
					switch (argv[i][2]) {
					case 'x':
						sizBoard.width = atoi(argv[i]+3);
						break;
					case 'y':
						sizBoard.height = atoi(argv[i]+3);
						break;
					case 's':
						fSqSz = atof(argv[i]+3);
						break;
					default:
						Usage( argv[0] );
					}
					break;
				case 'r':
					nRandom = atoi(argv[i] + 2);
					break;
				case 'o':
					pszOutPref = argv[i] + 2;
					break;
				case 'l':
					vProcListFile( imagePoints, sizImg, sizBoard, nWinSize, argv[i] + 2 );
					break;
				default:
					Usage( argv[0] );
				}
			}
			else {
				vAddImage( imagePoints, sizImg, sizBoard, nWinSize, argv[i] );
			}
		}

		if (nRandom >= 0) {
			vRandomizePoints( imagePoints, nRandom);
		}
		printf( "Frames = %ld\n", imagePoints.size() );

		Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
		Mat distCoeffs = Mat::zeros(8, 1, CV_64F);;	
	    vector<Mat> rvecs;
		vector<Mat> tvecs;
	    vector<float> reprojErrs;
	    double totalAvgErr = 0;
	    vector<Point3f> newObjPoints;

		float grid_width = fSqSz * (sizBoard.width - 1);
		vector<vector<Point3f> > objectPoints(1);
		calcBoardCornerPositions(sizBoard, fSqSz, objectPoints[0]);
		objectPoints[0][sizBoard.width - 1].x = objectPoints[0][0].x + grid_width;
		newObjPoints = objectPoints[0];

		objectPoints.resize(imagePoints.size(),objectPoints[0]);
		double dRMS;
#if 0
		dRMS = calibrateCameraRO(objectPoints, imagePoints, sizImg, -1,
								cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
								CALIB_USE_LU);
#else
		dRMS = calibrateCamera(objectPoints, imagePoints, sizImg,
								cameraMatrix, distCoeffs, rvecs, tvecs,
								CALIB_USE_LU);
#endif

		cout << cameraMatrix << endl;
		cout << distCoeffs << endl;
	}
	catch( exception &e ) {
		printf( "Error: %s\n", e.what() );
	}
#if 0
	fstream fs;	
	fs.open( pszInList, bWrite ? fstream::out : fstream::in );

	int nFrms = 0;
	cv::Mat frm;
	cv::Mat cameraMatrix, distCoeffs;

	int32_t i32Frm;
	int nExp;

	while ((i32Frm = vl.i32ReadNext( frm, nExp )) > 0) {
		nFrms++;
	}

	printf( "%d Frames\n", nFrms );
#endif
}
