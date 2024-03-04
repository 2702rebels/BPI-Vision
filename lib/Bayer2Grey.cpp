#include <assert.h>
#include <iostream>

#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


// MatGrey must be initialized to same size as matBayer and must be of type CV_8UC1
void Bayer2Grey( Mat &matGrey, Mat &matBayer )
{
	int nRows = matBayer.rows;
	int nLoopRows = nRows - 1;  // rows that have normal pixels .. i.e not special cases
	int nCols = matBayer.cols;
	int nLoopCols = nCols - 1;  // columns that have normal pixels .. i.e. not special cases
	
	assert( matGrey.rows == matBayer.rows && matGrey.cols == matBayer.cols );
	assert( matGrey.type() == CV_8UC1 && matBayer.type() == CV_8UC1 );
	assert( (matBayer.rows & 1) == 0 && (matBayer.cols & 1) == 0 );

	uint8_t *pu8Grey = matGrey.ptr( 0 );
	uint8_t *pu8Cur = matBayer.ptr( 0 );

	// Handle 1st row GRGR...
	*pu8Grey++ = pu8Cur[nCols];
	*pu8Grey++ = pu8Cur[0];
	*pu8Grey++ = pu8Cur[1];
	pu8Cur++;

	uint32_t u32Grey;

	for( int nCol = 1; nCol < nLoopCols; nCol++, pu8Cur++ ) {
		if (nCol & 1) {  // red
			u32Grey  = (pu8Cur[nCols-1] + pu8Cur[nCols+1]) / 2;
			u32Grey += (pu8Cur[-1] + pu8Cur[1] + pu8Cur[nCols]) / 3;
			u32Grey += pu8Cur[0];
		}
		else {
			u32Grey  = pu8Cur[nCols];
			u32Grey += pu8Cur[0];
			u32Grey += (pu8Cur[-1] + pu8Cur[1]) / 2;
		}
		*pu8Grey++ = u32Grey / 3;
	}

	u32Grey  = pu8Cur[nCols-1];
	u32Grey += (pu8Cur[-1] + pu8Cur[nCols]) / 2;
	u32Grey += pu8Cur[0];
	*pu8Grey++ = u32Grey / 3;
	pu8Cur++;

	//#pragma omp parallel for
	for (int nRow = 1; nRow < nLoopRows; nRow++ ) {
		pu8Cur = matBayer.ptr(nRow);
		pu8Grey = matGrey.ptr(nRow);

		if (nRow & 1) { // BGBGBG
			u32Grey  = pu8Cur[0];
			u32Grey += (pu8Cur[-nCols] + pu8Cur[1] + pu8Cur[nCols]) / 3;
			u32Grey += (pu8Cur[-nCols + 1] + pu8Cur[nCols + 1]) / 2;
		}
		else { // GRGRGR
			u32Grey  = (pu8Cur[-nCols] + pu8Cur[nCols]) / 2;
			u32Grey += pu8Cur[0];
			u32Grey += pu8Cur[1];
		}
		*pu8Grey++ = u32Grey / 3;
		pu8Cur++;
		for (int nCol = 1; nCol < nLoopCols; nCol++, pu8Cur++) {
			// The Bayer Pattern is a repeating 2x2 grid  of 
			// BG
			// GR
			// and there are 4 distinct algorithms for computing the missing colors
			// Determine pixel encoding type from least significant bits
			// the encoding will be G(00) R(01) B(10) G(11)

			uint8_t u8CT = ((nRow & 1) << 1) | (nCol & 1);
			switch( u8CT ) {
				case 0: // Green on Red Row
					u32Grey  = (pu8Cur[-nCols] + pu8Cur[nCols]) / 2;
					u32Grey += pu8Cur[0];
					u32Grey += (pu8Cur[-1] + pu8Cur[1]) / 2;
					break;
				case 1: // Red
					u32Grey  = (pu8Cur[-nCols -1] + pu8Cur[-nCols + 1] + pu8Cur[nCols - 1] + pu8Cur[nCols + 1]) / 4;
					u32Grey += (pu8Cur[-nCols] + pu8Cur[-1] + pu8Cur[1] + pu8Cur[nCols]) / 4;
					u32Grey += pu8Cur[0];
					break;
				case 2:  // Blue
					u32Grey  = pu8Cur[0];
					u32Grey += (pu8Cur[-nCols] + pu8Cur[-1] + pu8Cur[1] + pu8Cur[nCols]) / 4;
					u32Grey += (pu8Cur[-nCols -1] + pu8Cur[-nCols + 1] + pu8Cur[nCols - 1] + pu8Cur[nCols + 1]) / 4;
					break;
				case 3:  // Green on Blue Row
					u32Grey  = (pu8Cur[-1] + pu8Cur[1]) / 2;
					u32Grey += pu8Cur[0];
					u32Grey += (pu8Cur[-nCols] + pu8Cur[nCols]) / 2;
					break;
			}
			*pu8Grey++ = u32Grey / 3;
		}
		if (nRow & 1) { // BGBGBG...
			u32Grey  = pu8Cur[-1];
			u32Grey += pu8Cur[0];
			u32Grey += (pu8Cur[-nCols] + pu8Cur[nCols])/2;
		}
		else {  // GRGRGR...
			u32Grey  = (pu8Cur[-nCols -1] + pu8Cur[nCols - 1]) / 2;
			u32Grey += (pu8Cur[-nCols] + pu8Cur[-1] + pu8Cur[nCols]) / 3;
			u32Grey += pu8Cur[0];
		}
		*pu8Grey++ = u32Grey / 3;
		pu8Cur++;
	}

	// Need to handle last row...
}

// MatGrey must be initialized to same size as matBayer and must be of type CV_8UC1
void Bayer2HalfGrey( Mat &matGrey, Mat const &matBayer )
{
	assert( (matGrey.cols & 0x7) == 0 );
	assert( matGrey.rows == matBayer.rows/2 && matGrey.cols == matBayer.cols/2 );
	assert( matGrey.type() == CV_8UC1 && matBayer.type() == CV_8UC1 );
	assert( (matBayer.rows & 1) == 0 && (matBayer.cols & 1) == 0 );

	int nRows = matBayer.rows;
	int nCols = matBayer.cols;

	for( int nRow = 0; nRow < nRows; nRow += 2 ) {
		uint32_t const *pu32B1 = (uint32_t const *)matBayer.ptr(nRow);
		uint32_t const *pu32B2 = (uint32_t const *)matBayer.ptr(nRow + 1);
		uint32_t *pu32Gr = (uint32_t *)matGrey.ptr(nRow / 2 );

		for( int nCol = 0; nCol < nCols; nCol += 8 ) {
			uint32_t u32B1 = *pu32B1++;
			uint32_t u32B2 = *pu32B2++;
			uint32_t u32Res = (u32B1 & 0x00FF00FF) + ((u32B1 >> 8) & 0x00FF00FF) +
							  (u32B2 & 0x00FF00FF) + ((u32B2 >> 8) & 0x00FF00FF);
			u32Res >>= 2;

			uint32_t u32Grey = (u32Res & 0xff) | ((u32Res >> 8) & 0x0000ff00);

			u32B1 = *pu32B1++;
			u32B2 = *pu32B2++;
			u32Res = (u32B1 & 0x00FF00FF) + ((u32B1 >> 8) & 0x00FF00FF) +
					 (u32B2 & 0x00FF00FF) + ((u32B2 >> 8) & 0x00FF00FF);
			u32Res >>= 2;
			u32Grey |= ((u32Res << 16) & 0x00FF0000) | ((u32Res << 8) & 0xFF000000);

			*pu32Gr++ = u32Grey;
		}
	}
}

#if 0
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
	Mat matGrey( matBayer.rows, matBayer.cols, CV_8UC3 );
	Bayer2RGB( matGrey, matBayer );
	imwrite( argv[2], matGrey );
	return 0;
	
}

#endif
