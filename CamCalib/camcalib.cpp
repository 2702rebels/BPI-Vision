#include <stdlib.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <curses.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

#include "2702math.h"
#include "2702camera.h"

using namespace Rebels;

#include "stereoCalib.h"
#include "monoCalib.h"

#define YELLOW_ON_BLUE 1


void CursesInit()
{
	initscr();
	start_color();
	init_pair( YELLOW_ON_BLUE, COLOR_YELLOW, COLOR_BLUE );
	nonl();
	cbreak();
	noecho();
	attrset( A_NORMAL );
	clear();
	refresh();
}

void DisplayResults( vector<string> const &res )
{
	WINDOW  *pBorderWin = newwin( stdscr->_maxy - 4, stdscr->_maxx - 4, 2, 2 );
	wattrset( pBorderWin, A_NORMAL );
	wclear( pBorderWin );

	WINDOW  *pTextWin = subwin( pBorderWin, pBorderWin->_maxy-2, pBorderWin->_maxx-2, 1+pBorderWin->_begy, 1 + pBorderWin->_begx );
	wattrset( pTextWin, A_NORMAL );
	wclear( pTextWin );

	box( pBorderWin, '*', '*' );
	mvwaddstr( pBorderWin, 0, 5, "ESC to close" );
	wrefresh( pBorderWin );

	int nFirstRow = 0;
	bool bDone = false;
	bool bChanged = true;
	do {
		if( bChanged ) {
			int nMax = pTextWin->_maxy;
			if( nMax > res.size() ) {
				nMax = res.size();
			}

			for( int i = 0; i < nMax; i++ ) {
				mvwaddstr( pTextWin, i+1,1, res[i + nFirstRow].c_str() );
				wclrtoeol( pTextWin );
			}
			bChanged = false;
			wrefresh( pTextWin );
		}
		int ch = getch();
		switch( ch )
		{
		case 0x1b:
			bDone = true;
			break;
		case 'k':
			if( nFirstRow > 0 ) {
				bChanged = true;
				nFirstRow--;
			}
			break;
		case 'j':
			if( res.size() >  nFirstRow + pTextWin->_maxy) {
				bChanged = true;
				nFirstRow++;
			}
			break;
		default:
			break;
		}
	} while( !bDone );

	
	werase( pBorderWin );
	delwin( pTextWin );
	delwin( pBorderWin );
	touchwin( stdscr );
	refresh();
}

int nCreateRcvSocket()
{
  int srvrsock = socket(AF_INET, SOCK_DGRAM, 0);
  if (srvrsock < 0) {
    printf("Unable to create udp socket\n");
    return -1;
  }

  struct sockaddr_in udpaddr;
  memset(&udpaddr, 0, sizeof(udpaddr));
  udpaddr.sin_family = AF_INET;
  udpaddr.sin_addr.s_addr = INADDR_ANY;
  udpaddr.sin_port = htons(2702 + 16);
  if (bind(srvrsock, (sockaddr*)&udpaddr, sizeof(udpaddr)) < 0) {
    printf("Unable to bind udp socket\n");
    return -1;
  }

  return srvrsock;
}

void Usage()
{
	cerr << "Usage: camcalib -[1|2] -c#[,#] [-H#] -L# [-ooutfile] [-ppointfile] camfile" << endl;
	cerr << "\t-1          single camera calibration" << endl;
	cerr << "\t-2          stereo camera calibration" << endl;
	cerr << "\t-c#[,#]     camera number 1 [,camera number 2 for stereo]" << endl;
	cerr << "\t-H#         object height for single camera calibration" << endl;
	cerr << "\t-L#         length of calibration artifact" << endl;
	cerr << "\t-Y          align Y axis with 1st point" << endl;
	cerr << "\t-iinfile    input csv pointfile" << endl; 
	cerr << "\t-ooutfile   updated camera file is written to outfile instead of overwriting input file" << endl;
	cerr << "\t-ppointfile calibration point are stored to pointfile" << endl;
	cerr << "\tcamfile     inital (and output if no -o) camera file" << endl;
}

vector<vector<Point2d> > grCalPoints[2];
vector<Point2d> grCurPoints[2];


StereoCam	stereoCam;
CamParm		monoCam;

int grnCamNos[2] = { -1, -1 };

#define numCams (bStereo ? 2 : 1 )

void SavePoints( char const *psz, bool bStereo )
{
	ofstream of;
	of.open( psz );
	if( !of.is_open() ) {
		mvprintw( 1, 1, "Failed opening %s for output", psz );
		refresh();
		return;
	}
	for( int i = 0; i < grCalPoints[0].size(); i++ ) {
		for( int c = 0; c < numCams; c++ ) {
			for( int p = 0; p < grCalPoints[c][i].size(); p++ ) {
				char szTmp[1024];
				sprintf( szTmp, "%7.3f,%7.3f,", grCalPoints[c][i][p].u(), grCalPoints[c][i][p].v() );
				of << szTmp;
			}
		}
		of << endl;
	}
}

bool LoadPoints( char const *psz, bool bStereo )
{
	ifstream infile;
	infile.open( psz );
	if( !infile.is_open() ) {
		fprintf( stderr, "Unable to open %s for input\n", psz );
		return false;
	}
	
	char szTmp[1024];
	while( !infile.getline( szTmp, sizeof(szTmp) ).eof() )
	{
		int nOffset = 0;
		for( int cam = 0; cam < numCams; cam++ ) {
			vector<Point2d> pts;
			for( int tgt = 0; tgt < 2; tgt++ ) {
				float fU, fV;
				int nRead;
				
				if( 2 != sscanf( szTmp + nOffset, "%f,%f,%n", &fU, &fV, &nRead ) ) {
					cerr << "Failed parsing " << szTmp;
					return false;
				}
				Point2d p( fU, fV );
				pts.push_back( p );
				nOffset += nRead;
			}
			grCalPoints[cam].push_back(pts);
		}
	}
	return true;
}

int main( int argc, char **argv )
{
	bool bAlignYAxis = false;
	char const *pszPointFile = 0;
	char const *pszInCalibFile = 0;
	char const *pszOutCalibFile = 0;
	float fCalHeight = -100.f;
	float fCalLength = -100.f;
	bool bStereo = false;

	for( argc--, argv++; argc; argc--, argv++ ) {
		if( argv[0][0] == '-' )
		{
			switch( argv[0][1] ) {
				case '1':
					bStereo = false;
					break;
				case '2':
					bStereo = true;
					break;
				case 'c':
					if( bStereo ) {
						if( sscanf( argv[0]+2, "%d,%d", &grnCamNos[0], &grnCamNos[1] ) != 2 ) {
							fprintf( stderr, "Must specify 2 cameras (-c#,#) when performing stereo calibration (%s)", argv[0] );
							Usage();
							return 1;
						}
					}
					else {
						if( sscanf( argv[0]+2, "%d", &grnCamNos[0] ) != 1 ) {
							fprintf( stderr, "Unable to parse '%s'\n", argv[0] );
							Usage();
							return 1;
						}
					}
					break;
				case 'i':
					if( !LoadPoints( argv[0]+2, bStereo ) ) {
						Usage();
						return 1;
					}
					break;
				case 'p':
					pszPointFile = argv[0] + 2;
					break;
				case 'H':
					fCalHeight = atof( argv[0]+2 );
					break;
				case 'L':
				case 'l':
					fCalLength = atof( argv[0]+2 );
					break;
				case 'o':
					pszOutCalibFile = argv[0]+2;
					break;
				case 'Y':
					bAlignYAxis = true;
					break;
				default:
					Usage();
					break;
			}
		}
		else if( !pszInCalibFile ) {
			pszInCalibFile = argv[0];
		}
		else {
			Usage();
			return 1;
		}
	}
	if( !pszInCalibFile || fCalHeight < 0.F || fCalLength < 0.F || grnCamNos[0] < 0 || (bStereo && grnCamNos[1] < 0) ) {
		Usage();
		return 1;
	}

	if( bStereo ) {
		if( !stereoCam.bLoad( pszInCalibFile ) ) {
			cerr << "Unable to load " << pszInCalibFile <<endl;
			return 1;
		}
	}
	else {
		if( !monoCam.Load( pszInCalibFile ) ) {
			cerr << "Unable to load " << pszInCalibFile <<endl;
			return 1;
		}
	}

	struct sockaddr_in piAddr;

	int sockfd = nCreateRcvSocket();
	if( sockfd < 0 )
		return 1;

	CursesInit();

	mvprintw( 0, 1, "ESC - Exit   p - take point   d - delete last  c - calibrate   s - save" );
	nodelay( stdscr, 1 );
	mvprintw( 1, 0, "# Points: %d", (int)grCalPoints[0].size() );
  int len;
  bool bDone = false;
  do {
    char buf[2048];
    socklen_t nAddrLen = sizeof(piAddr);
    ssize_t nRecvd = recvfrom(sockfd, buf, sizeof(buf) - 1, MSG_DONTWAIT,
                              (sockaddr*)&piAddr, &nAddrLen);
    if (nRecvd <= 0) {
	  usleep( 10000 );
    }
	else if( nRecvd >= 2 && buf[0] == 27 && buf[1] == 2 )
	{
		int camidx = -1;
		int camno = buf[2];

		if( camno == grnCamNos[0] ) {
			camidx = 0;
		}
		else if( bStereo && camno == grnCamNos[1] ) {
			camidx = 1;
		}
		else {
			continue;
		}

		
		vector<Point2d> &pts = grCurPoints[camidx];
		pts.clear();

		int targets = buf[3];
		int nIdx = 4;
		for( int i = 0; i < targets; i++ )
		{
			float fU;
			float fV;
			memcpy( &fU, buf + nIdx, 4 );
			nIdx += 4;
			memcpy( &fV, buf + nIdx, 4 );
			nIdx += 4;

			pts.push_back( Point2d( fU, fV ) );
		}

		for( int i = -1; i < 4; i++ )
		{
			for( int c = 0; c < numCams; c++ )
			{
				char szTmp[32];

				if( i < 0 ) {
					sprintf( szTmp, "Camera %d    ", grnCamNos[c] );
				}
				else if( i < grCurPoints[c].size() ) {
					sprintf( szTmp, "%6.2f,%6.2f", grCurPoints[c][i].u(), grCurPoints[c][i].v() );
				}
				else {
					strcpy( szTmp, "                  " );
				}
				mvprintw( 4 + i, 1 + c * 20, "%s", szTmp );
			}
		}
		refresh();
	}
	
  	int ch = getch();
	switch( ch )
	{
		case 0x1b:
			bDone = true;
			break;
		case 'p':
			if( grCurPoints[0].size() != 2 || (bStereo && grCurPoints[1].size() != 2) ) {
				mvprintw( 1, 0, "Unable to take sample - don't have 2 points for each camera" );
				refresh();
				break;
			}
			for( int c = 0; c < numCams; c++ ) {
				grCalPoints[c].push_back( grCurPoints[c] );
			}
			mvprintw( 1, 0, "# Points: %d", (int)grCalPoints[0].size() );
			refresh();
			break;
		case 'd':
			if( grCalPoints[0].size() > 0 )
			{
				for( int c = 0; c < numCams; c++ ) {
					grCalPoints[c].pop_back();
				}
			}
			break;
		case 'c':
			if( bStereo && grCalPoints[0].size() > 4 ) {
				StereoCalib sc( stereoCam, grCalPoints[0], grCalPoints[1], bAlignYAxis );
				float fCal = sc.fCalibrate( fCalLength, fCalHeight );
				DisplayResults( sc.results() );
				sc.saveLog( "stereo.log" );
				mvprintw( 1, 20, "CalResult: %f", fCal );
				refresh();
			}
			else if( grCalPoints[0].size() > 5 ) {
				MonoCalib mc( monoCam, grCalPoints[0] );
				float fCal = mc.fCalibrate( fCalLength, fCalHeight );
				DisplayResults( mc.results() );
				mc.saveLog( "mono.log" );
				mvprintw( 1, 20, "CalResult: %f", fCal );
				refresh();
			}
			break;
		case 's':
			if( bStereo ) {
				stereoCam.Save( pszOutCalibFile ? pszOutCalibFile : pszInCalibFile );
			}
			else {
				monoCam.Save( pszOutCalibFile ? pszOutCalibFile : pszInCalibFile );
			}
			SavePoints( pszPointFile, bStereo );
			mvprintw( 1, 1, "Saved" );
			clrtoeol();
			refresh();
			break;
	}
  } while( !bDone );

  endwin();
  return 0;
}

