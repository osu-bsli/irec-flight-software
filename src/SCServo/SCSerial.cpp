/**
 * SCSerial.h
 * hardware interface layer for waveshare serial bus servo
 * date: 2023.6.28
 *
 * Modified to replace Arduino support with Linux support
 * @author Brian Jia
 * @date 2025.10.11
 */


#include "SCSerial.h"

#include "termios.h"
#include <cstddef>
#include <unistd.h>
#include <cstdio>

SCSerial::SCSerial()
{
	IOTimeOut = 100;
	SerialFD = -1;
}

SCSerial::SCSerial(u8 End):SCS(End)
{
	IOTimeOut = 100;
	SerialFD = -1;
}

SCSerial::SCSerial(u8 End, u8 Level):SCS(End, Level)
{
	IOTimeOut = 100;
	SerialFD = -1;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{
	ssize_t bytes_read = read(SerialFD, nDat, nLen);
	return bytes_read;
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	//printf("writing %d bytes\n", nLen);
	return write(SerialFD, nDat, nLen);
}

int SCSerial::writeSCS(unsigned char bDat)
{
	return write(SerialFD, &bDat, 1);
}

void SCSerial::rFlushSCS()
{
	//printf("rFlushSCS\n");
	tcflush(SerialFD, TCIFLUSH);
}

void SCSerial::wFlushSCS()
{
	//printf("wFlushSCS\n");
	tcdrain(SerialFD);
}
