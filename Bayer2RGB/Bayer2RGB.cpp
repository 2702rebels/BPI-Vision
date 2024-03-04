#include <assert.h>
#include <iostream>

#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


// MatRGB must be initialized to same size as matBayer and must be of type CV_8UC3
void Bayer2RGB( Mat &matRGB, Mat &matBayer )
{
	int nRows = matBayer.rows;
	int nLoopRows = nRows - 1;  // rows that have normal pixels .. i.e not special cases
	int nCols = matBayer.cols;
	int nLoopCols = nCols - 1;  // columns that have normal pixels .. i.e. not special cases
	
	assert( matRGB.rows == matBayer.rows && matRGB.cols == matBayer.cols );
	assert( matRGB.type() == CV_8UC3 && matBayer.type() == CV_8UC1 );
	assert( (matBayer.rows & 1) == 0 && (matBayer.cols & 1) == 0 );

	uint8_t *pu8BGR = matRGB.ptr( 0 );
	uint8_t *pu8Cur = matBayer.ptr( 0 );

	// Handle 1st row GRGR...
	*pu8BGR++ = pu8Cur[nCols];
	*pu8BGR++ = pu8Cur[0];
	*pu8BGR++ = pu8Cur[1];
	pu8Cur++;

	for( int nCol = 1; nCol < nLoopCols; nCol++, pu8Cur++ ) {
		if (nCol & 1) {  // red
			*pu8BGR++ = (pu8Cur[nCols-1] + pu8Cur[nCols+1]) / 2;
			*pu8BGR++ = (pu8Cur[-1] + pu8Cur[1] + pu8Cur[nCols]) / 3;
			*pu8BGR++ = pu8Cur[0];
		}
		else {
			*pu8BGR++ = pu8Cur[nCols];
			*pu8BGR++ = pu8Cur[0];
			*pu8BGR++ = (pu8Cur[-1] + pu8Cur[1]) / 2;
		}
	}

	*pu8BGR++ = pu8Cur[nCols-1];
	*pu8BGR++ = (pu8Cur[-1] + pu8Cur[nCols]) / 2;
	*pu8BGR++ = pu8Cur[0];
	pu8Cur++;

	//#pragma omp parallel for
	for (int nRow = 1; nRow < nLoopRows; nRow++ ) {
		pu8Cur = matBayer.ptr(nRow);
		pu8BGR = matRGB.ptr(nRow);

		if (nRow & 1) { // BGBGBG
			*pu8BGR++ = pu8Cur[0];
			*pu8BGR++ = (pu8Cur[-nCols] + pu8Cur[1] + pu8Cur[nCols]) / 3;
			*pu8BGR++ = (pu8Cur[-nCols + 1] + pu8Cur[nCols + 1]) / 2;
		}
		else { // GRGRGR
			*pu8BGR++ = (pu8Cur[-nCols] + pu8Cur[nCols]) / 2;
			*pu8BGR++ = pu8Cur[0];
			*pu8BGR++ = pu8Cur[1];
		}
		pu8Cur++;
		for (int nCol = 1; nCol < nLoopCols; nCol++, pu8Cur++) {
			// The Bayer Pattern is a repeating 2x2 grid  of 
			// GR
			// BG
			// and there are 4 distinct algorithms for computing the missing colors
			// Determine pixel encoding type from least significant bits
			// the encoding will be G(00) R(01) B(10) G(11)

			uint8_t u8CT = ((nRow & 1) << 1) | (nCol & 1);
			switch( u8CT ) {
				case 0: // Green on Red Row
					*pu8BGR++ = (pu8Cur[-nCols] + pu8Cur[nCols]) / 2;
					*pu8BGR++ = pu8Cur[0];
					*pu8BGR++ = (pu8Cur[-1] + pu8Cur[1]) / 2;
					break;
				case 1: // Red
					*pu8BGR++ = (pu8Cur[-nCols -1] + pu8Cur[-nCols + 1] + pu8Cur[nCols - 1] + pu8Cur[nCols + 1]) / 4;
					*pu8BGR++ = (pu8Cur[-nCols] + pu8Cur[-1] + pu8Cur[1] + pu8Cur[nCols]) / 4;
					*pu8BGR++ = pu8Cur[0];
					break;
				case 2:  // Blue
					*pu8BGR++ = pu8Cur[0];
					*pu8BGR++ = (pu8Cur[-nCols] + pu8Cur[-1] + pu8Cur[1] + pu8Cur[nCols]) / 4;
					*pu8BGR++ = (pu8Cur[-nCols -1] + pu8Cur[-nCols + 1] + pu8Cur[nCols - 1] + pu8Cur[nCols + 1]) / 4;
					break;
				case 3:  // Green on Blue Row
					*pu8BGR++ = (pu8Cur[-1] + pu8Cur[1]) / 2;
					*pu8BGR++ = pu8Cur[0];
					*pu8BGR++ = (pu8Cur[-nCols] + pu8Cur[nCols]) / 2;
					break;
			}
		}
		if (nRow & 1) { // BGBGBG...
			*pu8BGR++ = pu8Cur[-1];
			*pu8BGR++ = pu8Cur[0];
			*pu8BGR++ = (pu8Cur[-nCols] + pu8Cur[nCols])/2;
		}
		else {  // GRGRGR...
			*pu8BGR++ = (pu8Cur[-nCols -1] + pu8Cur[nCols - 1]) / 2;
			*pu8BGR++ = (pu8Cur[-nCols] + pu8Cur[-1] + pu8Cur[nCols]) / 3;
			*pu8BGR++ = pu8Cur[0];
		}
		pu8Cur++;
	}

	// Need to handle last row...
}

int main( int argc, char **argv )
{
	if (argc !=3) {
		cerr << "Usage: Bayer2RGB infile outfile\n";
		return 1;
	}
	Mat matBayer = imread( argv[1], IMREAD_UNCHANGED );
	if (matBayer.empty()) {
		cerr << "Unable to open input file\n";
		return 1;
	}
	
	cout << "Image Type: " << matBayer.type() << "  Expect " << CV_8UC1 << endl;
	cout << "Image Size: " << matBayer.rows << " x " << matBayer.cols << endl;
	Mat matRGB( matBayer.rows, matBayer.cols, CV_8UC3 );
	Bayer2RGB( matRGB, matBayer );
	imwrite( argv[2], matRGB );
	return 0;
	
}

