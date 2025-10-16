// C library headers
#include <stdio.h>
#include <string.h>

// WiringPi headers
#include <wiringPi.h>
#include <wiringSerial.h>

#include <unistd.h>
#include <termios.h>
#include <SCServo.h>

#include <curses.h>

#define SERVO_SERIAL_PORT "/dev/ttyS0"
#define SERVO_SERIAL_BAUD 1000000
#define SERVO_POWER_EN_GPIO 18

SMS_STS sms_sts;

void servo_setup();
void servo_move_test();
void servo_deinit();

int main()
{
	servo_setup();
	servo_move_test();
	servo_deinit();

	return 0;
}

void servo_setup()
{
	wiringPiSetupGpio(); // Initializes wiringPi using the Broadcom GPIO pin numbers
	pinMode(SERVO_POWER_EN_GPIO, OUTPUT); 
	digitalWrite(SERVO_POWER_EN_GPIO, HIGH);

	int serial_port = serialOpen(SERVO_SERIAL_PORT, SERVO_SERIAL_BAUD);

    // Check for errors
    if (serial_port < 0) {
		printf("[FAILED] Failed to open serial port\n");
	}   
	else
	{
		printf("[  OK  ] Successfully opened serial port!\n");
	}

	tcflush(serial_port, TCIOFLUSH);
	sms_sts.SerialFD = serial_port;
	
	sleep(1);
}

void servo_move_test()
{
	initscr(); // Initialize the curses library
    cbreak();  // Line buffering disabled, pass characters immediately to program
    noecho();  // Don't echo input characters
    keypad(stdscr, TRUE); // Enable keypad mode for special keys

    int ch;
    printw("Press the Up and Down arrow keys to adjust position (or 'q' to quit):\n");

	int pos = 2000;
	int speed = 100;
	
	printw("Position (0-4095): %d\n", pos);

    while ((ch = getch()) != 'q')
    {
        if (ch == KEY_UP)
        {
            pos += 10;
          	if (pos > 4000)
          	{
          		pos = 4000;
          	}
          	move(1, 0);
        	printw("Position (0-4095): %d\n", pos);
		} 
		else if (ch == KEY_DOWN)
		{
	    	pos -= 10;
	    	if (pos < 0)
	    	{
	    		pos = 0;
	    	}
	    	move(1, 0);
        	printw("Position (0-4095): %d\n", pos);
        }
        else if (ch == KEY_RIGHT)
        {
            speed += 10;
          	if (speed > 1000)
          	{
          		speed = 1000;
          	}
          	move(2, 0);
        	printw("Speed: %d\n", speed);
		} 
		else if (ch == KEY_LEFT)
		{
	    	speed -= 10;
	    	if (speed < 0)
	    	{
	    		speed = 0;
	    	}
	    	move(2, 0);
        	printw("Speed: %d\n", speed);
        }
        else
        {
            printw("Key pressed: %d\n", ch);
        }

        sms_sts.WritePosEx(1, pos, speed, speed);
    }

    endwin(); // End curses mode
}

void servo_deinit()
{
	serialClose(sms_sts.SerialFD);
}
