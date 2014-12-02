#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>

using namespace std;

int main()
{
    cout << "pangs";
    int USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );
    cout << "pangs";
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( USB, &tty ) != 0 )
    {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
    }

    /* Save old tty parameters */
    tty_old = tty;

    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B9600);
    cfsetispeed (&tty, (speed_t)B9600);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;        // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;       // no flow control
    tty.c_cc[VMIN]      =   1;                  // read doesn't block
    tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    cout << "test" << flush;
    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( USB, TCIFLUSH );
    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0)
    {
        cout << "Error " << errno << " from tcsetattr" << endl;
    }

    //WRITE
    unsigned char cmd[] = "INIT \r";
    int n_written = 0;
    cout << "write" << flush;
    do 
    {
        n_written += write( USB, &cmd[n_written], 1 );
    }
    while (cmd[n_written-1] != '\r' && n_written > 0);
	cout << "read" << flush;
    //READ
    int n = 0;
    char buf = '\0';

    /* Whole response*/
    std::string response;

    do
    {
       cout << "1" << flush;
       n = read( USB, &buf, 1 );
       cout << "2" << flush;
       response.append( &buf );
    }
    while( buf != '\r' && n > 0);
    cout << "dicks" << flush;
    if (n < 0)
    {
       cout << "Error reading: " << strerror(errno) << endl;
    }
       else if (n == 0)
    {
        cout << "Read nothing!" << endl;
    }
    else
    {
        cout << "Response: " << response;
    }
    return 0;
}
