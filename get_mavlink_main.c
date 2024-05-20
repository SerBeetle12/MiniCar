#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include "string.h"

#include "common/mavlink.h"

#include "term.h"

#define home() 			printf(ESC "[H") //Move cursor to the indicated row, column (origin at 1,1)
#define clrscr()		printf(ESC "[2J") //lear the screen, move to (1,1)
#define gotoxy(x,y)		printf(ESC "[%d;%dH", y, x);
#define visible_cursor() printf(ESC "[?251");

#define resetcolor() printf(ESC "[0m")
#define set_display_atrib(color) 	printf(ESC "[%dm",color)


mavlink_msg_rc_main_type msg_rc_main;
mavlink_msg_rc_power_type msg_rc_power;
mavlink_status_t status;

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
//           0123456789012345678901234567890123456789
	puts(	"┌───────╥─────────────────╥────────────┐\n" //0
			"│  ADC  ║      POWER      ║    SBUS    │\n" //1
			"│ 1:    ║ IN      V     A ║ CH 1       │\n" //2
			"│ 2:    ║ 5V      V     A ║ CH 2       │\n" //3
			"│ 3:    ║ 5V2     V     A ║ CH 3       │\n" //4
			"│ 4:    ║    DISCHARGE    ║ CH 4       │\n" //5
			"│       ╠═════════════════╣ CH 5       │\n" //6
			"│ B1    ║   TEMPERATURE   ║ CH 6       │\n" //7
			"│ B2    ║          C°     ║ CH 7       │\n" //8
			"│ B3    ╠═════════════════╣ CH 8       │\n" //9
			"│ B4    ║  Joystick NVH   ║ CH 9       │\n" //10
			"│ BH    ║      ____       ║ CH 10      │\n" //11
			"│       ║      _UP_       ║ CH 11      │\n" //12
			"│       ║ ____       ____ ║ CH 12      │\n" //13
			"│       ║ LEFT VALID RIGH ║ CH 13      │\n" //14
			"│       ║ ____       ____ ║ CH 14      │\n" //15
			"│       ║      ____       ║ CH 15      │\n" //16
			"│       ║      DOWN       ║ CH 16      │\n" //17
			"│       ║                 ║ 17/18      │\n" //18
			"└───────╨─────────────────╨────────────┘\n");  //19
	resetcolor();
}

void print_adc(unsigned char data, unsigned char num)
{
	gotoxy(5, 3+num);
	set_display_atrib(BRIGHT);
	set_display_atrib(B_BLACK);
	set_display_atrib(F_GREEN);
	printf("%3d", data);
}

void print_btn(unsigned char data, unsigned char num)
{
	gotoxy(6, 8+num);
	set_display_atrib(BRIGHT);
	set_display_atrib(B_GREEN);
	set_display_atrib(F_WHITE);
	if(data)
	{
		set_display_atrib(B_GREEN);
		printf("%2d",  1);
	}
	else
	{
		set_display_atrib(B_BLACK);
		printf("%2d",  0);
	}
}

void print_NVH(unsigned char state, unsigned char num)
{
	set_display_atrib(F_WHITE);
	set_display_atrib(B_GREEN);
	switch(num)
	{
		case 0: //east
			gotoxy(22, 14);
			if(state)
			{
				
				printf("    ");
				gotoxy(22, 15);
				printf("RIGH");
				gotoxy(22, 16);
				printf("    ");
			}
			else
			{
				set_display_atrib(B_BLACK);
				printf("    ");
				gotoxy(22, 15);
				printf("RIGH");
				gotoxy(22, 16);
				printf("    ");
			}
		break;
		case 1:
			gotoxy(16, 12);
			if(state)
			{
				
				printf("     ");
				gotoxy(16, 13);
				printf(" UP  ");
			}
			else
			{
				set_display_atrib(B_BLACK);
				printf("     ");
				gotoxy(16, 13);
				printf(" UP  ");
			}
		break;
		case 2:
			gotoxy(11, 14);
			if(state)
			{
				
				printf("    ");
				gotoxy(11, 15);
				printf("LEFT");
				gotoxy(11, 16);
				printf("    ");
			}
			else
			{
				set_display_atrib(B_BLACK);
				printf("    ");
				gotoxy(11, 15);
				printf("LEFT");
				gotoxy(11, 16);
				printf("    ");
			}
		break;
		case 3:
			gotoxy(16, 17);
			if(state)
			{
				
				printf("     ");
				gotoxy(16, 18);
				printf("DOWN ");
			}
			else
			{
				set_display_atrib(B_BLACK);
				printf("     ");
				gotoxy(16, 18);
				printf("DOWN ");
			}
		break;
		case 4: //validation
		gotoxy(16, 15);
		if(state)
		{
			printf("VALID");
		}
		else
		{
			set_display_atrib(B_BLACK);
			printf("VALID");
		}
	break;
	}
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
	
	unsigned char uart_recieve_byte;
	unsigned char uart_recieve_start_byte_flag = 0;
	unsigned char uart_wait_msg = 1;
	unsigned char mavlink_len = 0;
	int recieve_cnt = 1000;
	
	mavlink_message_t msg = {0};
	
	clrscr();
	frame_draw();
	gotoxy(0, 21);
	
	char buffer_str[50];
	char buffer_str10[10];
	
	float f1 = 0.0f;
	float f2 = 0.0f;
	unsigned int byte = 0;
	
	printf("\e[?25l");
	
	while(recieve_cnt)
	{
		//clrscr();
		//frame_draw();
		
		msg.msgid = 0;
		uart_recieve_start_byte_flag = 0;
		uart_wait_msg = 1;
		mavlink_len = 0;
		
		while(uart_wait_msg)
		{
			uart_recieve_byte = get_byte(fd);
			//printf("b = %X\n", uart_recieve_byte);
			//header byte
			if(uart_recieve_byte == 0xFD && !uart_recieve_start_byte_flag)
			{
				//printf("Recieved header byte\n");
				mavlink_parse_char(0, uart_recieve_byte, &msg, &status);
				uart_recieve_start_byte_flag = 1;
				continue;
			}

			if(uart_recieve_start_byte_flag == 1)
			{
				mavlink_len = uart_recieve_byte + 10;
				uart_recieve_start_byte_flag = 2;
				//printf("Mavlink length message = %d\n", mavlink_len);
			}

			if(uart_recieve_start_byte_flag == 2)
			{
				mavlink_len--;
				if(mavlink_parse_char(0, uart_recieve_byte, &msg, &status))
				{
					uart_wait_msg = 0;
				}
				if(!mavlink_len)
					break;
			}
		}
		
		
		
		if(msg.msgid == 602001)
		{
			//printf("msgid %d\n", msg.msgid);
			
			mavlink_msg_rc_main_decode(&msg, &msg_rc_main);
		
			print_adc(msg_rc_main.knupel_1_X, 0);
			print_adc(msg_rc_main.knupel_1_Y, 1);
			print_adc(msg_rc_main.knupel_2_X, 2);
			print_adc(msg_rc_main.knupel_2_Y, 3);

			print_btn(msg_rc_main.button_1, 0);
			//print_btn(msg_rc_main.button_2, 1);
			print_btn(msg_rc_main.button_3, 2);
			print_btn(msg_rc_main.button_4, 3);
			print_btn(msg_rc_main.button_HOME, 4);

			print_NVH(msg_rc_main.NVH_East, 0);
			print_NVH(msg_rc_main.NVH_South, 3);
			print_NVH(msg_rc_main.NVH_West, 2);
			print_NVH(msg_rc_main.NVH_North, 1);
			print_NVH(msg_rc_main.NVH_Validation, 4);
		}
		
		if(msg.msgid == 602003)
		{
			
			//printf("msgid %d\n", msg.msgid);
			
			mavlink_msg_rc_power_decode(&msg, &msg_rc_power);
			
			//printf("input voltage = %f\n", msg_rc_power.ina_in_voltage);

			gotoxy(15, 3);
			set_display_atrib(BRIGHT);
			set_display_atrib(B_BLACK);
			set_display_atrib(F_GREEN);
			
			f1 = msg_rc_power.ina_in_voltage;
			f2 = msg_rc_power.ina_in_current;
			if(f1 < 30.0f && f1 > 0 && f2 < 10.0f && f2 > 0)
				printf("%1.1fV %1.2fA", f1, f2);
			//sprintf(buffer_str, "%02.1fV %1.2fA", msg_rc_power.ina_in_voltage, msg_rc_power.ina_in_current);
			//strncpy(buffer_str10, buffer_str, 10);
			//printf("%10s", buffer_str);
			
			gotoxy(16, 4);
			f1 = msg_rc_power.ina_5V_voltage;
			f2 = msg_rc_power.ina_5V_current;
			if(f1 < 10.0f && f1 > 0 && f2 < 10.0f && f2 > 0)
				printf("%1.1fV %1.2fA", f1, f2);
			//sprintf(buffer_str, "%1.1fV %1.2fA", msg_rc_power.ina_5V_voltage, msg_rc_power.ina_5V_current);
			//strncpy(buffer_str10, buffer_str, 10);
			//printf("%10s", buffer_str);
			
			gotoxy(16, 5);
			f1 = msg_rc_power.ina_5V2_voltage;
			f2 = msg_rc_power.ina_5V2_current;
			if(f1 < 10.0f && f1 > 0 && f2 < 10.0f && f2 > 0)
				printf("%1.1fV %1.2fA", f1, f2);
			
			gotoxy(13, 6);
			byte = msg_rc_power.state;
			if(byte > 0 && byte < 10)
				printf("   ST %d   ", msg_rc_power.state);
			
			//printf("%1.1fV %1.2fA", msg_rc_power.ina_5V_voltage, msg_rc_power.ina_5V_current);
			//gotoxy(16, 5);
			//printf("%1.1fV %-3.2fA", msg_rc_power.ina_5V2_voltage, msg_rc_power.ina_5V2_current);
		}
		
		
		
		
		gotoxy(0, 21);
	}
	
	
	
	
	
	//printf("Result msg_id = %d \n", msg.msgid);
	//printf("Result msg_knupel_1_X = %d \n", msg_rc_main.knupel_1_X);
	//printf("Result msg_button_1 = %d \n", msg_rc_main.button_1);
	
	
	
	gotoxy(0, 21);

	close(fd);
}

//gcc get_mavlink_main.c -o test_get_main -Wno-address-of-packed-member

