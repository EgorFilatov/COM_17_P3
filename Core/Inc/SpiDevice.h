#include "stm32f0xx.h"

#ifndef SRC_SPIDEVICE_H_
#define SRC_SPIDEVICE_H_

enum State {
	READY,
	BUSY,
    READY_FOR_SPI_TX_RX, 	// Готов для приемо-передачи по SPI
    IN_SPI_TX_RX,        	// Находится в состоянии приемо-передачи по SPI
    READY_FOR_PROCESSING,	// Готов для обработки данных
    IN_PROCESSING,  		// Находится в состоянии обработки данных
    READY_FOR_UART_RX, 		// Готов к приему по UART
    IN_UART_RX,   			// Находится в состоянии приема по UART
    READY_FOR_UART_TX,		// Готов к передаче по UART
    IN_UART_TX 				// Находится в состоянии передачи по UART
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
	/* Приемный буфер:
	 0:		Размер
	 1:		Контрольная сумма
	 2:		Данные/инструкция
	 3:		Номер порта
	 4:		Тип платы
	 5-...:	Данные */
	uint8_t rxBuff[9] { 0 };
	uint8_t previousRxBuff[9] { 0 };

	/* Буфер для передачи:
	 0:		Размер
	 1:		Контрольная сумма
	 2:		Данные/инструкция
	 3:		Номер порта
	 4:		Тип платы
	 5-...:	Данные */
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
