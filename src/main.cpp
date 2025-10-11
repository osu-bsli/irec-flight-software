// C library headers
#include <stdio.h>
#include <string.h>

// WiringPi headers
#include <wiringPi.h>
#include <wiringSerial.h>

#include <unistd.h>
#include <SCServo.h>

#define SERVO_SERIAL_PORT "/dev/ttyS0"
#define SERVO_SERIAL_BAUD 1000000
#define SERVO_POWER_EN_GPIO 18

SMS_STS sms_sts;

void servo_setup();
void servo_move_test();

int main()
{
	servo_setup();
	servo_move_test();

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
	sms_sts.SerialFD = serial_port;
	
	sleep(1);
}

void servo_move_test()
{
	printf("Performing movement test\n");

	sms_sts.WritePosEx(1, 4095, 3400, 50);//servo(ID1) speed=3400，acc=50，move to position=4095.
	usleep(1000*2000);
  
	sms_sts.WritePosEx(1, 2000, 1500, 50);//servo(ID1) speed=3400，acc=50，move to position=2000.
	usleep(1000*2000);
}
