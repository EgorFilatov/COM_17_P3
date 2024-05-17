#include <SpiDevice.h>

uint8_t SpiDevice::devicesNumber { 0 };
uint8_t SpiDevice::currentDeviceIndex { 0 };
uint8_t SpiDevice::currentBuffIndex { 0 };
State SpiDevice::rxBuffState[2] { READY_FOR_SPI_TX_RX };
State SpiDevice::txBuffState[2] { READY_FOR_SPI_TX_RX };

SpiDevice::SpiDevice() {
	index = devicesNumber;
	++devicesNumber;
	state = NOT_AVAILIABLE;
	type = NOT_DEFINED;
	csPort = GPIOA;
	csPin = 0;
}

void SpiDevice::setCS(GPIO_TypeDef *port, uint8_t pin) {
	csPort = port;
	csPin = pin;
}

uint8_t* SpiDevice::getTxBuffPtr(uint8_t buffIndex) {
	if (buffIndex == 0) {
		return txBuff;
	}
	return &txBuff[8];
}

uint8_t* SpiDevice::getRxBuffPtr(uint8_t buffIndex) {
	if (buffIndex == 0) {
		return rxBuff;
	}
	return &rxBuff[8];
}

void SpiDevice::select() {
	csPort->BRR |= (1 << csPin);
}

void SpiDevice::deselect() {
	csPort->BSRR |= (1 << csPin);
}

uint8_t SpiDevice::verifyRxChecksum(uint8_t buffIndex) {
	uint32_t sum { 0 };
	for (uint8_t i = buffIndex * 8 + 2; i <= buffIndex * 8 + 7; ++i) {
		sum += rxBuff[i];
	}
	return (sum == rxBuff[buffIndex * 8 + 1]) ? 1 : 0;
}

uint8_t SpiDevice::isChanged(uint8_t buffIndex) {
	uint8_t buffStart = buffIndex * 8;
	uint8_t buffEnd = start + 7;
	for (uint8_t i = buffStart; i < buffEnd; ++i) {
		if (rxBuff[i] != previousRxBuff[i]) {
			for (uint8_t i = buffStart; i < buffEnd; ++i) {
				previousRxBuff[i] = rxBuff[i];
			}
			return 1;
		}
	}
	return 0;
}


uint8_t SpiDevice::getCurrentDeviceIndex() {
	return currentDeviceIndex;
}
void SpiDevice::setCurrentDeviceIndex(uint8_t index) {
	currentDeviceIndex = index;
}
void SpiDevice::increaseCurrentDeviceIndex() {
	++currentDeviceIndex;
}
void SpiDevice::decreaseCurrentDeviceIndex() {
	--currentDeviceIndex;
}

uint8_t SpiDevice::getCurrentBuffIndex() {
	return currentBuffIndex;
}

void SpiDevice::setCurrentBuffIndex(uint8_t index) {
	currentBuffIndex = index;
}
void SpiDevice::toggleCurrentBuffIndex() {
	currentBuffIndex ^= 1;
}

void SpiDevice::setRxBuffState(uint8_t buffIndex, State state) {
	rxBuffState[buffIndex] = state;
}
State SpiDevice::getRxBuffState(uint8_t buffIndex) {
	 return rxBuffState[buffIndex];
}

void SpiDevice::setTxBuffState(uint8_t buffIndex, State state) {
	txBuffState[buffIndex] = state;
}
State SpiDevice::getTxBuffState(uint8_t buffIndex) {
	 return txBuffState[buffIndex];
}

uint8_t SpiDevice::getDevicesNumber() {
	return devicesNumber;
}

DeviceState SpiDevice::getState() {
	return state;

}
void SpiDevice::setState(DeviceState state) {
	this->state = state;
}

DeviceType SpiDevice::getType() {
	return type;
}
void SpiDevice::setType(DeviceType type) {
	this->type = type;
}






