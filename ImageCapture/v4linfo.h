#include "v4lcam-mp.h"


class v4linfo : public v4lcam
{
public:
	v4linfo( char const *psz ) : v4lcam( psz )
	{
	}
	virtual ~v4linfo()
	{
	}
	void printCaps()
	{
		v4l2_capability  cap;
		if( ioctl( m_fd, VIDIOC_QUERYCAP, &cap ) < 0 )
			throw invalid_argument( string("invalid video device") );

		printf( "Driver: %s\n", (char *)cap.driver );
		printf( "card:   %s\n", (char *)cap.card );
		printf( "ver:    %08x\n", cap.version );
		printf( "caps:   %08x\n", cap.capabilities );
		printf( "devcaps:%08x\n", cap.device_caps );
	}
	void printStandards()
	{
		v4l2_input	input;
		v4l2_standard standard;

		memset( &input, 0, sizeof(input) );

		if( ioctl( m_fd, VIDIOC_G_INPUT, &input.index ) < 0 )
			throw invalid_argument( string("VIDIOC_G_INPUT") );

		if( ioctl( m_fd, VIDIOC_ENUMINPUT, &input ) < 0 )
			throw invalid_argument( string("VIDIOC_ENUMINPUT") );

		printf( "Current input %s supports:\n", input.name );

		memset( &standard, 0, sizeof(standard) );
		standard.index = 0;

		while( ioctl( m_fd, VIDIOC_ENUMSTD, &standard ) == 0 )
		{
			if( standard.id & input.std )
				printf( "%s\n", standard.name );
			standard.index++;
		}
	}
	void enumerateMenu( v4l2_query_ext_ctrl &ctrl )
	{
		v4l2_querymenu menu;

		printf( "  Menu items:\n" );
		memset( &menu, 0, sizeof(menu) );
		menu.id = ctrl.id;

		for( menu.index = ctrl.minimum; menu.index < ctrl.maximum; menu.index++ )
		{
			if( !ioctl( m_fd, VIDIOC_QUERYMENU, &menu ) )
				printf( "    %-3d %s\n", menu.index, menu.name );
		}
	}
	void printControls()
	{
		v4l2_query_ext_ctrl	queryctrl;
		memset( &queryctrl, 0, sizeof(queryctrl) );
		queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;

		while( !ioctl( m_fd, VIDIOC_QUERY_EXT_CTRL, &queryctrl ) )
		{
			if( !(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) )
			{
				printf( "Control %08x - %s\n", queryctrl.id, queryctrl.name );

				if( queryctrl.type == V4L2_CTRL_TYPE_MENU )
					enumerateMenu( queryctrl );
			}
			queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
		}
	}
};
