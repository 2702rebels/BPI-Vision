/*****************************************************************
Name:               GenIO

Description:
    This module defines routines used with Curses and the API
    for handling general I O functionality.

Library Name:

Include Files:

Modifications:
    Created:    Walter Carpini  Dec. 1993

*****************************************************************/

/*****************************************************************
C Library Files Included
*****************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <curses.h>
#include <stdarg.h>
#include <ctype.h>
#include <time.h>

/*****************************************************************
ND Library Files Included
*****************************************************************/

/*****************************************************************
Project Files Included
*****************************************************************/
#include "genio.h"

/*****************************************************************
Global Variables
*****************************************************************/
static char szMessageLine[ 1024 ];

/***************************************************************************

Routine:           InitScreen

Inputs:
    None

Returns:
    void

Description:
    Initializes the screen through curses.

***************************************************************************/
void InitScreen( void )
{
    initscr();
    nonl();
    cbreak();
    noecho();
    attrset( A_NORMAL );
    clear();
    refresh();

} /* InitScreen */

/*****************************************************************
Name:               ErrorMsg

Input Values:
    char
        *pszFmt         : Format string.

Output Values:
    none.
Returned Value:
    none.

Description:
    Call this routine to output an error message box to the screen
    and wait for Keyboard input.

*****************************************************************/
void ErrorMsg( char const *pszFmt, ... )
{
    WINDOW  *pErrWin;
    WINDOW  *pTxtWin;
    va_list
        va;

    pErrWin = newwin( 10, 50, (stdscr->_maxy - 10) / 2,
                      (stdscr->_maxx - 50) / 2 );
    pTxtWin = subwin( pErrWin, 5, 46, 3 + pErrWin->_begy, 2 + pErrWin->_begx );
    wattrset( pErrWin, A_NORMAL );
    wclear( pErrWin );
    wattrset( pTxtWin, A_NORMAL );
    wclear( pTxtWin );
    box( pErrWin, '*', '*' );
    mvwaddstr( pErrWin, 1, 18, "Error Message" );
    va_start( va, pszFmt );
    vsprintf( szMessageLine, pszFmt, va );
    mvwaddstr( pTxtWin, 0, 0, szMessageLine );
    mvwaddstr( pErrWin, 8, 12, "Press Any Key to Continue" );
    wrefresh( pErrWin );
    
    nodelay( stdscr, 0 );
    getch();
    nodelay( stdscr, 1 );

    werase( pErrWin );
    delwin( pTxtWin );
    delwin( pErrWin );
    touchwin( stdscr );
    refresh();

} /* ErrorMsg */

/*****************************************************************
Name:               WarningMsg

Input Values:
    char
        *pszFmt         : Format string.

Output Values:
    none.

Returned Value:
    none.

Description:
    Call this routine to output warning message box to the screen
    and wait for Keyboard input.

*****************************************************************/
void WarningMsg( char const *pszFmt, ... )
{
    WINDOW  *pErrWin;
    WINDOW  *pTxtWin;
    va_list
        va;

    pErrWin = newwin( 10, 50, (stdscr->_maxy - 10) / 2,
                      (stdscr->_maxx - 50) / 2 );
    pTxtWin = subwin( pErrWin, 5, 46, 3 + pErrWin->_begy, 2 + pErrWin->_begx );
    wattrset( pErrWin, A_NORMAL );
    wclear( pErrWin );
    wattrset( pTxtWin, A_NORMAL );
    wclear( pTxtWin );
    box( pErrWin, '*', '*' );
    mvwaddstr( pErrWin, 1, 18, "Warning Message" );
    va_start( va, pszFmt );
    vsprintf( szMessageLine, pszFmt, va );
    mvwaddstr( pTxtWin, 0, 0, szMessageLine );
    mvwaddstr( pErrWin, 8, 12, "Press Any Key to Continue" );
    wrefresh( pErrWin );

    nodelay( stdscr, 0 );
    getch();
    nodelay( stdscr, 1 );

    werase( pErrWin );
    delwin( pTxtWin );
    delwin( pErrWin );
    touchwin( stdscr );
    refresh();
} /* WarningMessage */

/*****************************************************************
Name:               GetYesNo

Input Values:
    char
        *pszFmt         : Format string.

Output Values:
    none.

Returned Value:
    none.

Description:
    Call this routine to output a message box to the screen
    and wait for Keyboard input of yes or no.

*****************************************************************/
bool GetYesNo( char const *pszFmt, ... )
{
    WINDOW  *pErrWin;
    WINDOW  *pTxtWin;
    int     ch;
    va_list va;

    pErrWin = newwin( 10, 50, (stdscr->_maxy - 10) / 2,
                      (stdscr->_maxx - 50) / 2 );
    pTxtWin = subwin( pErrWin, 5, 46, 3 + pErrWin->_begy, 2 + pErrWin->_begx );
    wattrset( pErrWin, A_NORMAL );
    wclear( pErrWin );
    wattrset( pTxtWin, A_NORMAL );
    wclear( pTxtWin );
    box( pErrWin, '*', '*' );
    mvwaddstr( pErrWin, 1, 18, "Error Message" );
    va_start( va, pszFmt );
    vsprintf( szMessageLine, pszFmt, va );
    mvwaddstr( pTxtWin, 0, 0, szMessageLine );
    mvwaddstr( pErrWin, 8, 12, "Press Y or N to continue." );
    wrefresh( pErrWin );

    do {
        ch = getch();
        if( islower( ch ) )
            ch = toupper( ch );
    } while( ch != 'Y' && ch != 'N' );

    werase( pErrWin );
    delwin( pTxtWin );
    delwin( pErrWin );
    touchwin( stdscr );
    refresh();
    return (ch == 'Y') ? true : false;
} /* GetYesNo */
