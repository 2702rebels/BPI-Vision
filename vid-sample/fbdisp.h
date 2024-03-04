#ifndef FBDISP_H
#define FBDISP_H

#include <fcntl.h>
#include <linux/fb.h>

class fbDisp {
	int			m_fd;
	uint8_t		*m_pu8Mem;
	int			m_nRows;
	int			m_nCols;
	int			m_nLeft;
	int			m_nTop;
	uint32_t	m_u32Size;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;

public:
	fbDisp( int nLeft, int nTop, int nRows, int nCols );
	~fbDisp();

	void Display( cv::Mat const &mat );
	void Display( cv::Mat const &mat, int nLeft, int nTop );
};

#endif
