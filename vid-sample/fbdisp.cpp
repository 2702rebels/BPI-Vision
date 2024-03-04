
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <string>
#include <stdexcept>

using namespace std;

#include "fbdisp.h"

fbDisp::fbDisp( int nLeft, int nTop, int nRows, int nCols ) :
	m_nLeft( nLeft ),
	m_nTop( nTop ),
	m_nRows( nRows ),
	m_nCols( nCols ),
	m_pu8Mem( 0 )
{
	m_fd = open("/dev/fb0", O_RDWR);
	if( m_fd < 0 )
		throw runtime_error( string("Failed opening frame buffer") );

	if (ioctl(m_fd, FBIOGET_FSCREENINFO, &finfo) == -1)
		throw runtime_error( string("Failed getting FSCREENINFO") );

	if (ioctl(m_fd, FBIOGET_VSCREENINFO, &vinfo) == -1)
		throw runtime_error( string("Failed getting VSCREENINFO") );

	// Figure out the size of the screen in bytes
	m_u32Size = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;


	// Map the device to memory
	m_pu8Mem = (uint8_t *)mmap(0, m_u32Size, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0);
	if ((int)m_pu8Mem == -1)
		throw runtime_error( string("Failed mapping Frame Buffer memory") );


	printf( "Screen Size: %d x %d\n", vinfo.xres, vinfo.yres );
	printf( "Bits/pixel = %d\n", vinfo.bits_per_pixel );
}

fbDisp::~fbDisp()
{
	if( m_pu8Mem )
		munmap(m_pu8Mem, m_u32Size);
	if( m_fd >= 0 )
		close(m_fd);
}

void fbDisp::Display( cv::Mat const &mat )
{
	int nRows = mat.rows;
	int nCols = mat.cols;
	
	if( mat.type() == CV_8UC1 )
	{
		for( int row = 0; row < nRows; row++ )
		{
			const uint8_t *src = mat.ptr(row,0);
			uint8_t *dst = m_pu8Mem + (m_nLeft + vinfo.xoffset) * vinfo.bits_per_pixel/8 +
				(row + vinfo.yoffset + m_nTop) * finfo.line_length;
			for( int col = 0; col < nCols; col++ )
			{
				*dst++ = 0; // Blue
				*dst++ = *src++; // Green
				*dst++ = 0; // Red
				*dst++ = 0; // Transparency
			}
		}
	}
	else
	{
		for( int row = 0; row < nRows; row++ )
		{
			const uint8_t *src = mat.ptr(row,0);
			uint8_t *dst = m_pu8Mem + (m_nLeft + vinfo.xoffset) * vinfo.bits_per_pixel/8 +
				(row + vinfo.yoffset + m_nTop) * finfo.line_length;

			for( int col = 0; col < nCols; col++ )
			{
				*dst++ = *src++; // Blue
				*dst++ = *src++; // Green
				*dst++ = *src++; // Red
				*dst++ = 0; // Transparency
			}
		}
	}
}

void fbDisp::Display( cv::Mat const &mat, int nLeft, int nTop )
{
	int nRows = mat.rows;
	int nCols = mat.cols;
	
	if( mat.type() == CV_8UC1 )
	{
		for( int row = 0; row < nRows; row++ )
		{
			const uint8_t *src = mat.ptr(row,0);
			uint8_t *dst = m_pu8Mem + (nLeft + m_nLeft + vinfo.xoffset) * vinfo.bits_per_pixel/8 +
				(row + vinfo.yoffset + nTop + m_nTop) * finfo.line_length;
			uint32_t *pu32 = (uint32_t *)dst;
			for( int col = 0; col < nCols; col++ )
			{
				//*dst++ = 0; // Blue
				//*dst++ = *src++; // Green
				//*dst++ = 0; // Red
				//*dst++ = 0; // Transparency

				uint32_t u32 = (*src++);
				u32 |= ((u32 << 8) | (u32 << 16));
				*pu32++ = u32;
			}
		}
	}
	else
	{
		for( int row = 0; row < nRows; row++ )
		{
			const uint8_t *src = mat.ptr(row,0);
			uint8_t *dst = m_pu8Mem + (nLeft + m_nLeft + vinfo.xoffset) * vinfo.bits_per_pixel/8 +
				(row + vinfo.yoffset + nTop + m_nTop) * finfo.line_length;

			for( int col = 0; col < nCols; col++ )
			{
				*dst++ = *src++; // Blue
				*dst++ = *src++; // Green
				*dst++ = *src++; // Red
				*dst++ = 0; // Transparency
			}
		}
	}
}
