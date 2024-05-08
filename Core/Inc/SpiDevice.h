#include "stm32f0xx.h"

#ifndef SRC_SPIDEVICE_H_
#define SRC_SPIDEVICE_H_

enum State {
	READY,
	BUSY,
    READY_FOR_SPI_TX_RX, 	// ����� ��� ������-�������� �� SPI
    IN_SPI_TX_RX,        	// ��������� � ��������� ������-�������� �� SPI
    READY_FOR_PROCESSING,	// ����� ��� ��������� ������
    IN_PROCESSING,  		// ��������� � ��������� ��������� ������
    READY_FOR_UART_RX, 		// ����� � ������ �� UART
    IN_UART_RX,   			// ��������� � ��������� ������ �� UART
    READY_FOR_UART_TX,		// ����� � �������� �� UART
    IN_UART_TX 				// ��������� � ��������� �������� �� UART
};

enum DeviceState {
	NOT_AVAILIABLE,
	ACTIVE,
    DATA_CHANGED
};

enum DeviceType {
	NOT_DEFINED,
	I20_G4_SS,
	O16NO_G2T,
	O20NO_G4T
};

class SpiDevice {
private:
	/* �������� �����:
	 0:		������
	 1:		����������� �����
	 2:		������/����������
	 3:		����� �����
	 4:		��� �����
	 5-...:	������ */
	uint8_t rxBuff[9] { 0 };
	uint8_t previousRxBuff[9] { 0 };

	/* ����� ��� ��������:
	 0:		������
	 1:		����������� �����
	 2:		������/����������
	 3:		����� �����
	 4:		��� �����
	 5-...:	������ */
	uint8_t txBuff[9] { 0 };
	uint8_t previousTxBuff[9] { 0 };

	uint8_t index;
	DeviceState state;
	DeviceType type;
	GPIO_TypeDef *csPort;
	uint8_t csPin;

	static uint8_t devicesNumber;
	static uint8_t currentDeviceIndex;
	static uint8_t currentBuffIndex;
	static State rxBuffState[2];
	static State txBuffState[2];


public:
	SpiDevice();
	void setCS(GPIO_TypeDef *port, uint8_t pin);

	uint8_t* getTxBuffPtr(uint8_t buffIndex);
	uint8_t* getRxBuffPtr(uint8_t buffIndex);

	void select();
	void deselect();

	uint8_t verifyRxChecksum(uint8_t buffIndex);
	uint8_t isChanged(uint8_t buffIndex);

	static uint8_t getCurrentDeviceIndex();
	static void setCurrentDeviceIndex(uint8_t index);
	static void increaseCurrentDeviceIndex();
	static void decreaseCurrentDeviceIndex();

	static uint8_t getCurrentBuffIndex();
	static void setCurrentBuffIndex(uint8_t index);
	static void toggleCurrentBuffIndex();

	static void setRxBuffState(uint8_t buffIndex, State state);
	static State getRxBuffState(uint8_t buffIndex);

	static void setTxBuffState(uint8_t buffIndex, State state);
	static State getTxBuffState(uint8_t buffIndex);

	static uint8_t getDevicesNumber();

	DeviceState getState();
	void setState(DeviceState state);

	DeviceType getType();
	void setType(DeviceType type);
};

#endif
