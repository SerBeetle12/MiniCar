#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include "string.h"

#include "term.h"

#define home() 			printf(ESC "[H") //Move cursor to the indicated row, column (origin at 1,1)
#define clrscr()		printf(ESC "[2J") //lear the screen, move to (1,1)
#define gotoxy(x,y)		printf(ESC "[%d;%dH", y, x);
#define visible_cursor() printf(ESC "[?251");

#define resetcolor() printf(ESC "[0m")
#define set_display_atrib(color) 	printf(ESC "[%dm",color)

unsigned char get_byte(int fd_input)
{
	unsigned char byte;
	long cnt = 3000000;
	
	while(!read(fd_input, &byte, 1) && (cnt-- > 0)){usleep(1);}
	if(cnt == 0)
		return  0;
	return  byte;
}

void frame_draw () {
	home();
	set_display_atrib(B_BLACK);
	set_display_atrib(F_YELLOW);
//           0123456789012345678901234
	puts(	"┌───────────────────────┐\n" //0
			"│     Accelerometer     │\n" //1
			"├───────┬───────┬───────┤\n" //2
			"│   x   │   y   │   z   │\n" //3
			"│ 16000 │ 16000 │ 16000 │\n" //4
			"╞═══════╧═══════╧═══════╡\n" //5
			"│       Gyroscope       │\n" //6
			"├───────┬───────┬───────┤\n" //7
			"│   x   │   y   │   z   │\n" //8
			"│ 16000 │ 16000 │ 16000 │\n" //9
			"╞═══════╧═══════╧═══════╡\n" //10
			"│     Magnetometer      │\n" //11
			"├───────┬───────┬───────┤\n" //12
			"│   x   │   y   │   z   │\n" //13
			"│ 16000 │ 16000 │ 16000 │\n" //14
			"└───────────────────────┘\n" ); //15
	resetcolor();
}


int main(int argc, char* argv[])
{
	int fd;
	fd = open("/dev/ttyS3", O_RDWR | O_NOCTTY );
	if (fd <0)
	{
		printf("Error opening port\n");
		return -1;
	}

	//Start setting port
	struct termios tty;
	tcgetattr(fd, &tty);
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;
	cfsetispeed(&tty, B115200);
	tcsetattr(fd, TCSANOW, &tty);
	//End setting port
	
	printf("Waiting message\n");
	
	clrscr();
	frame_draw();
	
	//printf("\e[?25l"); //off cursor

	close(fd);
}
