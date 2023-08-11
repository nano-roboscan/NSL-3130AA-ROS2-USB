#include "rclcpp/rclcpp.hpp"
#include <communication_constants.h>
#include "serial_connection.h"
#include "util.h"
#include <termios.h>
//#include <duration.hpp>


namespace com_lib
{

SerialConnection::SerialConnection():
expectedSize(0),
fileID(0)
{    
    dataArray.resize(307325);
}

SerialConnection::~SerialConnection()
{
    if(fileID != 0)
        closePort();        

    deviceListString.clear();    

}

/**
 * @brief Open the serial port
 *
 * @param id Id of the serial port = index of the list when the function "availableDevices" is called
 * @retval true Port is open
 * @retval false Port is not open
 * @return Port open or not
 */
bool SerialConnection::openPort(std::string portName)
{  
    if(fileID > 0){
        closePort();                
        usleep(100000);
    }

	bool bFind = false;
	char path[100];
	sprintf(path,"/dev/ttyLidar");
	fileID = open(path, O_RDWR | O_NOCTTY | O_SYNC); //ttyUSB0 -old
		
	if(fileID > 0)
	{
		bFind = true;
	}

	if( bFind == false ){
		printf("tty error ----------------------------------\n");
		return false;
	}

    printf ("openPort: %s  fd= %d\n", portName.c_str(), fileID);
    setInterfaceAttribs(B4000000);  // set speed to 10000000 bps, 8n1 (no parity)    

	flushRx();
    return true;
}

/**
 * @brief Close the port
 *
 */
void SerialConnection::closePort()
{              
    printf("closePort: fd = %d\n", fileID);
    fileID = close(fileID);
}


/**
 * @brief Send a command
 *
 * This function sends a command. It adds the marking and the checksum and sends it
 * to the device.
 *
 * Important: The size of the buffer must be big enough to add the markings an the checksum.
 *
 * @param data Pointer to the data to send
 */
ssize_t SerialConnection::sendData(uint8_t *data, int dataLen)
{
	static uint8_t sendbuf[com_const::Uart::COMMAND_BUFFER_SIZE];

    if(fileID <= 0 ){
        printf("Error SerialConnection::sendData fileID = 0 \n");
        return 0;
    }

//	printf("send cmd = %d\n", data[1]);

    //Add the start command mark at the beginning
    sendbuf[0] = static_cast<uint8_t>((com_const::Uart::START_MARK >> 24) & 0xFF);
    sendbuf[1] = static_cast<uint8_t>((com_const::Uart::START_MARK >> 16) & 0xFF);
    sendbuf[2] = static_cast<uint8_t>((com_const::Uart::START_MARK >> 8) & 0xFF);
    sendbuf[3] = static_cast<uint8_t>((com_const::Uart::START_MARK >> 0) & 0xFF);

    sendbuf[4] = static_cast<uint8_t>((dataLen >> 24) & 0xFF);
    sendbuf[5] = static_cast<uint8_t>((dataLen >> 16) & 0xFF);
    sendbuf[6] = static_cast<uint8_t>((dataLen >> 8) & 0xFF);
    sendbuf[7] = static_cast<uint8_t>((dataLen >> 0) & 0xFF);

	memcpy( &sendbuf[8], data, dataLen );

    //Add the end mark at the end
    sendbuf[com_const::Uart::COMMAND_BUFFER_SIZE-4] = static_cast<uint8_t>((com_const::Uart::END_MARK >> 24) & 0xFF);
    sendbuf[com_const::Uart::COMMAND_BUFFER_SIZE-3] = static_cast<uint8_t>((com_const::Uart::END_MARK >> 16) & 0xFF);
    sendbuf[com_const::Uart::COMMAND_BUFFER_SIZE-2] = static_cast<uint8_t>((com_const::Uart::END_MARK >> 8) & 0xFF);
    sendbuf[com_const::Uart::COMMAND_BUFFER_SIZE-1] = static_cast<uint8_t>((com_const::Uart::END_MARK >> 0) & 0xFF);

    /*std::string str= "SEND DATA: ";char buf[4];
    for(int i=0; i<39; i++){
        sprintf(buf, "%x ", data[i]); str.append(buf);
    }ROS_DEBUG_STREAM(str); //This is just to print out the data*/

    return write(fileID, sendbuf, com_const::Uart::COMMAND_BUFFER_SIZE);
}

/**
 * @brief Extract the expected size from the received data
 *
 * @param array Pointer to the received data
 * @return Expected size
 */
int SerialConnection::getExpextedSize(uint8_t* array)
{
    return Util::getUint32BigEndian(array, com_const::Uart::INDEX_DATA_SIZE);
}

/**
 * @brief Extract the type from the received data
 *
 * @param array Pointer to the received data
 * @return Received type
 */
uint8_t SerialConnection::getDataType(uint8_t* array)
{
    return array[com_const::Uart::INDEX_DATA_TYPE];
}

uint16_t SerialConnection::getImageType(uint8_t* array)
{
    return Util::getUint16BigEndian(array, com_const::Uart::INDEX_IMAGE_TYPE);
}

uint8_t SerialConnection::getType()
{
    uint8_t type = 0;

	type = getDataType(rxArray);
	if( type == com_const::Uart::VALUE_DATA_MEASUREMENT ){
		type = getImageType(rxArray);
        if(type == 3) type = com_const::Type::DATA_GRAYSCALE;
        else if(type == 0) type = com_const::Type::DATA_DISTANCE_AMPLITUDE;
        else if(type == 1) type = com_const::Type::DATA_DISTANCE;
	}
	else{
        if(rxArray[9] == 2){
            type = 2;
        }else if(rxArray[9] == 3){
            type = 3;
        }else if(rxArray[9] == 4){
            type = 4;
        }
	}

    return type;
}



bool SerialConnection::checkEndMark(uint8_t * array, int size)
{
    int i = size - 4;
//	0xFFFF55AA

    if(array[i] == 0xff && array[i+1] == 0xff && array[i+2] == 0x55 && array[i+3] == 0xaa)
        return true;

    return false;
}



int SerialConnection::setInterfaceAttribs(int speed)
{
    tcflush(fileID, TCIOFLUSH);

    struct termios tty;
    memset (&tty, 0, sizeof tty);

    tcgetattr (fileID, &tty); //TODO...

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    // no canonical processing
    // disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars

    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_oflag &= ~(ONLCR | OCRNL); //TODO...

    tty.c_lflag = 0;                // no signaling chars, no echo,
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); //TODO...

    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_iflag &= ~(INLCR | IGNCR | ICRNL); //TODO...

    tty.c_cc[VMIN]  = 0;            // non-blocking read
    tty.c_cc[VTIME] = 5;            // 0.5 second read timeout

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars                
    tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
    tty.c_cflag &= ~CSTOPB;    //one stop bit
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls   
    tty.c_cflag |= CRTSCTS;   //data DTR hardware control do not use it

    tcflush(fileID, TCIOFLUSH);

    if (tcsetattr (fileID, TCSANOW, &tty) != 0){
        printf("error %d from tcsetattr\n", errno);
        throw 3;
    }
   
    return 0;
}


bool SerialConnection::processData(int size)
{    
    //Check for the marking byte
	uint32_t startMark = Util::getUint32BigEndian(rxArray, 0);
    if(startMark != com_const::Uart::START_MARK){
        printf("SerialConnection::processData error com_const::Uart::START_MARK\n");

		if( size < 40 ){
			for(int i = 0;i<size;i++){
				printf(" %02x", rxArray[i]);
			}
			printf("\n");
		}
        return false;
    }

    if(checkEndMark(rxArray, size) == false){
        printf("SerialConnection::processData error checkEndMark size = %d\n", size);
//		for(int i = 0;i<size;i++){
//			printf(" %02x", rxArray[i]);
//		}
		printf("\n");
        return false;
    }

    //Get the expected size. Cancel here if not enough bytes received
    if (size < (static_cast<int>(com_const::Uart::SIZE_HEADER))){
        printf("SerialConnection::processData error sizeHeader\n");
        return false;
    }

    //Get the expexted size
    expectedSize = getExpextedSize(rxArray);

    //Cancel here if not enough bytes received
    if (size < (static_cast<int>(expectedSize + com_const::Uart::DATA_OVERHEAD_SIZE))){
        printf("SerialConnection::processData error size= %d\n", size);
        return false;
    }

    uint8_t type = getType();

    if((int)dataArray.size() != expectedSize && expectedSize > 1){
//        printf("SerialConnection::processData dataArray.size()= %d  expectedSize = %d\n", (int)dataArray.size(), expectedSize);
        dataArray.resize(expectedSize);
    }
        
    std::copy(rxArray + com_const::Uart::INDEX_DATA_PAYLOAD, rxArray + com_const::Uart::INDEX_DATA_PAYLOAD + expectedSize, dataArray.begin());

//	printf("::expectedSize = %d, full-size = %d\n", expectedSize, size);
	
    sigReceivedData(dataArray, type);
	dataArray.clear();

    return true;
}

int SerialConnection::flushRx(void)
{
	uint8_t buf[5000];
	int n = 0;
	int readflushData = 0;
//	const auto& time_cap0 = std::chrono::steady_clock::now();

	while(true)
	{
		n = read(fileID, buf, 4096);

		if(n > 0){
			readflushData += n;
		}else if( n == -1 ){
			printf("flush Error on  SerialConnection::readRxData= -1\n");
			break;
		}else if( n == 0 ){
			printf("flush readData %d bytes\n", readflushData);
			break;
		}

	}

	return 0;
}



/**
 * @brief Slot called on reception
 *
 * This function is called, when data from the serial port has been received.
 */
ErrorNumber_e SerialConnection::readRxData(int size)
{    
    uint8_t buf[4096];
    int n = 0;

//	const auto& time_cap0 = std::chrono::steady_clock::now();

    for(int i=0; i< size; i+=n)
    {
        memset(buf, 0, sizeof(buf)); //clear buffer

        unsigned long int buf_size = size;
        if(buf_size > sizeof(buf))
            buf_size = sizeof(buf);

        n = read(fileID, buf, buf_size);

        if(n > 0){                        
            memcpy(rxArray + i, buf, n);
        }else if(n == -1){
            printf("Error on  SerialConnection::readRxData= -1\n");
			openPort("");
            return ERROR_NUMBER_SERIAL_PORT_ERROR;
        }else if(n == 0 && i < size-1){
            printf("serialConnection->readRxData %d bytes from %d received\n", i, size);
			
			openPort("");
            return ERROR_NUMBER_SERIAL_PORT_ERROR;
        }

    }

//	const auto& time_cap1 = std::chrono::steady_clock::now();
//	double time_cam = (time_cap1 - time_cap0).count() / 1000000.0;
//	printf("  serial-Rx:		   %9.3lf [msec] len = %d\n", time_cam, size);


    processData(size);
    
    return ERROR_NUMMBER_NO_ERROR;
}



} //end namespace com_lib
