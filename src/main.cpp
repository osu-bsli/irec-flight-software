// C library headers
#include <stdio.h>
#include <string.h>

// WiringPi headers
#include <wiringPi.h>
#include <wiringSerial.h>

#include <unistd.h>
#include <termios.h>
#include <SCServo.h>

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
	printf("Performing movement test\n");

	sms_sts.WritePosEx(1, 4095, 20000, 100);//servo(ID1) speed=3400，acc=50，move to position=4095.
	printf("Sleeping for 2 seconds...\n");
	usleep(1000*2000);
  
	sms_sts.WritePosEx(1, 2000, 20000, 100);//servo(ID1) speed=3400，acc=50，move to position=2000.
	printf("Sleeping for 2 seconds...\n");
	usleep(1000*2000);
}

void servo_deinit()
{
	serialClose(sms_sts.SerialFD);
}
