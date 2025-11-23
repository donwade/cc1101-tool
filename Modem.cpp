/*
 *   Copyright (C) 2011-2018,2020,2021,2025 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

///#include "DStarDefines.h"
//#include "P25Defines.h"
///#include "YSFDefines.h"
#include "P25Defines.h"
///#include "NXDNDefines.h"
///#include "POCSAGDefines.h"
#include "Thread.h"
#include "Modem.h"
#include "Utils.h"
#include "Log.h"

#include <cmath>
#include <cstdio>
#include <cassert>
#include <cstdint>
#include <ctime>

#if defined(_WIN32) || defined(_WIN64)
#include <Windows.h>
#else
#include <unistd.h>
#endif

const unsigned char MMDVM_FRAME_START = 0xE0U;

const unsigned char MMDVM_GET_VERSION = 0x00U;
const unsigned char MMDVM_GET_STATUS  = 0x01U;
const unsigned char MMDVM_SET_CONFIG  = 0x02U;
const unsigned char MMDVM_SET_MODE    = 0x03U;
const unsigned char MMDVM_SET_FREQ    = 0x04U;

const unsigned char MMDVM_SEND_CWID   = 0x0AU;

const unsigned char MMDVM_DMR_DATA1   = 0x18U;
const unsigned char MMDVM_DMR_LOST1   = 0x19U;
const unsigned char MMDVM_DMR_DATA2   = 0x1AU;
const unsigned char MMDVM_DMR_LOST2   = 0x1BU;
const unsigned char MMDVM_DMR_SHORTLC = 0x1CU;
const unsigned char MMDVM_DMR_START   = 0x1DU;
const unsigned char MMDVM_DMR_ABORT   = 0x1EU;

const unsigned char MMDVM_P25_HDR     = 0x30U;
const unsigned char MMDVM_P25_LDU     = 0x31U;
const unsigned char MMDVM_P25_LOST    = 0x32U;

const unsigned char MMDVM_POCSAG_DATA = 0x50U;

const unsigned char MMDVM_ACK         = 0x70U;
const unsigned char MMDVM_NAK         = 0x7FU;

const unsigned char MMDVM_SERIAL_DATA = 0x80U;

const unsigned char MMDVM_TRANSPARENT = 0x90U;
const unsigned char MMDVM_QSO_INFO    = 0x91U;

const unsigned char MMDVM_DEBUG1      = 0xF1U;
const unsigned char MMDVM_DEBUG2      = 0xF2U;
const unsigned char MMDVM_DEBUG3      = 0xF3U;
const unsigned char MMDVM_DEBUG4      = 0xF4U;
const unsigned char MMDVM_DEBUG5      = 0xF5U;
const unsigned char MMDVM_DEBUG_DUMP  = 0xFAU;

const unsigned int MAX_RESPONSES = 30U;

const unsigned int BUFFER_LENGTH = 2000U;

const unsigned char CAP1_DMR    = 0x02U;
const unsigned char CAP1_P25    = 0x08U;
const unsigned char CAP2_POCSAG = 0x01U;


CModem::CModem(bool duplex, bool rxInvert, bool txInvert, bool pttInvert, unsigned int txDelay, unsigned int dmrDelay, bool useCOSAsLockout, bool trace, bool debug) :
m_protocolVersion(0U),
m_p25TXHang(5U),
 m_duplex(duplex),
m_rxInvert(rxInvert),
m_txInvert(txInvert),
m_pttInvert(pttInvert),
m_txDelay(txDelay),
m_rxLevel(0.0F),
m_cwIdTXLevel(0.0F),
m_dmrTXLevel(0.0F),
m_p25TXLevel(0.0F),
 m_pocsagTXLevel(0.0F),
m_rfLevel(0.0F),
m_useCOSAsLockout(useCOSAsLockout),
m_trace(trace),
m_debug(debug),
m_rxFrequency(0U),
m_txFrequency(0U),
m_pocsagFrequency(0U),
m_p25Enabled(false),
m_pocsagEnabled(false),
m_rxDCOffset(0),
m_txDCOffset(0),
m_port(nullptr),
m_buffer(nullptr),
m_length(0U),
m_offset(0U),
m_state(SERIAL_STATE::START),
m_type(0U),
m_rxDMRData1(1000U, "Modem RX DMR1"),
m_rxDMRData2(1000U, "Modem RX DMR2"),
m_txDMRData1(1000U, "Modem TX DMR1"),
m_txDMRData2(1000U, "Modem TX DMR2"),
m_rxYSFData(1000U, "Modem RX YSF"),
m_txYSFData(1000U, "Modem TX YSF"),
m_rxP25Data(1000U, "Modem RX P25"),
m_txP25Data(1000U, "Modem TX P25"),
m_txPOCSAGData(1000U, "Modem TX POCSAG"),
m_rxSerialData(1000U, "Modem RX Serial"),
m_txSerialData(1000U, "Modem TX Serial"),
m_rxTransparentData(1000U, "Modem RX Transparent"),
m_txTransparentData(1000U, "Modem TX Transparent"),
m_sendTransparentDataFrameType(0U),
m_statusTimer(1000U, 0U, 250U),
m_inactivityTimer(1000U, 2U),
m_playoutTimer(1000U, 0U, 10U),
m_p25Space(0U),
m_pocsagSpace(0U),
m_tx(false),
m_cd(false),
m_lockout(false),
m_error(false),
m_mode(MODE_IDLE),
m_hwType(HW_TYPE::UNKNOWN),
m_capabilities1(0x00U),
m_capabilities2(0x00U),
m_serialDataLen(0U)
{
	m_buffer = new unsigned char[BUFFER_LENGTH];
}

CModem::~CModem()
{
	delete   m_port;
	delete[] m_buffer;
}

void CModem::setPort(IModemPort* port)
{
	assert(port != nullptr);

	m_port = port;
}

void CModem::setRFParams(unsigned int rxFrequency, int rxOffset, unsigned int txFrequency, int txOffset, int txDCOffset, int rxDCOffset, float rfLevel, unsigned int pocsagFrequency)
{
	m_rxFrequency     = rxFrequency + rxOffset;
	m_txFrequency     = txFrequency + txOffset;
	m_txDCOffset      = txDCOffset;
	m_rxDCOffset      = rxDCOffset;
	m_rfLevel         = rfLevel;
	m_pocsagFrequency = pocsagFrequency + txOffset;
}

void CModem::setModeParams(bool p25Enabled, bool pocsagEnabled)
{
	m_p25Enabled    = p25Enabled;
 	m_pocsagEnabled = pocsagEnabled;
}

void CModem::setLevels(float rxLevel, float cwIdTXLevel, float dmrTXLevel, float p25TXLevel, float pocsagTXLevel)
{
	m_rxLevel       = rxLevel;
	m_cwIdTXLevel   = cwIdTXLevel;
	m_dmrTXLevel    = dmrTXLevel;
	m_p25TXLevel    = p25TXLevel;
 	m_pocsagTXLevel = pocsagTXLevel;
}

void CModem::setP25Params(unsigned int txHang)
{
	m_p25TXHang = txHang;
}

void CModem::setTransparentDataParams(unsigned int sendFrameType)
{
    m_sendTransparentDataFrameType = sendFrameType;
}

bool CModem::open()
{
	::LogMessage("Opening the MMDVM");

	bool ret = m_port->open();
	if (!ret)
		return false;

	ret = readVersion();
	if (!ret) {
		m_port->close();
		delete m_port;
		m_port = nullptr;
		return false;
	} else {
		/* Stopping the inactivity timer here when a firmware version has been
		   successfuly read prevents the death spiral of "no reply from modem..." */
		m_inactivityTimer.stop();
	}

	ret = setFrequency();
	if (!ret) {
		m_port->close();
		delete m_port;
		m_port = nullptr;
		return false;
	}

	ret = writeConfig();
	if (!ret) {
		m_port->close();
		delete m_port;
		m_port = nullptr;
		return false;
	}

	m_statusTimer.start();

	m_error  = false;
	m_offset = 0U;

	return true;
}

void CModem::clock(unsigned int ms)
{
	assert(m_port != nullptr);

	// Poll the modem status every 250ms
	m_statusTimer.clock(ms);
	if (m_statusTimer.hasExpired()) {
		readStatus();
		m_statusTimer.start();
	}

	m_inactivityTimer.clock(ms);
	if (m_inactivityTimer.hasExpired()) {
		LogError("No reply from the modem for some time, resetting it");
		m_error = true;
		close();

		CThread::sleep(2000U);		// 2s
		while (!open())
			CThread::sleep(5000U);	// 5s
	}

	RESP_TYPE_MMDVM type = getResponse();

	if (type == RESP_TYPE_MMDVM::TIMEOUT) {
		// Nothing to do
	} else if (type == RESP_TYPE_MMDVM::ERR) {
		// Nothing to do
	} else {
		// type == OK
		switch (m_type) {

			case MMDVM_P25_HDR: {
				if (m_trace)
					CUtils::dump(1U, "RX P25 Header", m_buffer, m_length);

				unsigned char data = m_length - m_offset + 1U;
				m_rxP25Data.addData(&data, 1U);

				data = TAG_HEADER;
				m_rxP25Data.addData(&data, 1U);

				m_rxP25Data.addData(m_buffer + m_offset, m_length - m_offset);
			}
			break;

			case MMDVM_P25_LDU: {
				if (m_trace)
					CUtils::dump(1U, "RX P25 LDU", m_buffer, m_length);

				unsigned char data = m_length - m_offset + 1U;
				m_rxP25Data.addData(&data, 1U);

				data = TAG_DATA;
				m_rxP25Data.addData(&data, 1U);

				m_rxP25Data.addData(m_buffer + m_offset, m_length - m_offset);
			}
			break;

			case MMDVM_P25_LOST: {
				if (m_trace)
					CUtils::dump(1U, "RX P25 Lost", m_buffer, m_length);

				unsigned char data = 1U;
				m_rxP25Data.addData(&data, 1U);

				data = TAG_LOST;
				m_rxP25Data.addData(&data, 1U);
			}
			break;

 			case MMDVM_GET_STATUS:
				// if (m_trace)
				//	CUtils::dump(1U, "GET_STATUS", m_buffer, m_length);

				switch (m_protocolVersion) {
				case 1U: {
						m_mode = m_buffer[m_offset + 1U];

						m_tx = (m_buffer[m_offset + 2U] & 0x01U) == 0x01U;
						bool adcOverflow = (m_buffer[m_offset + 2U] & 0x02U) == 0x02U;
						if (adcOverflow)
							LogError("MMDVM ADC levels have overflowed");
						bool rxOverflow = (m_buffer[m_offset + 2U] & 0x04U) == 0x04U;
						if (rxOverflow)
							LogError("MMDVM RX buffer has overflowed");
						bool txOverflow = (m_buffer[m_offset + 2U] & 0x08U) == 0x08U;
						if (txOverflow)
							LogError("MMDVM TX buffer has overflowed");
						m_lockout = (m_buffer[m_offset + 2U] & 0x10U) == 0x10U;
						bool dacOverflow = (m_buffer[m_offset + 2U] & 0x20U) == 0x20U;
						if (dacOverflow)
							LogError("MMDVM DAC levels have overflowed");
						m_cd = (m_buffer[m_offset + 2U] & 0x40U) == 0x40U;

						m_p25Space    = 0U;
 						m_pocsagSpace = 0U;

						// The following depend on the version of the firmware
						if (m_length > (m_offset + 7U))
							m_p25Space    = m_buffer[m_offset + 7U];
 						if (m_length > (m_offset + 9U))
							m_pocsagSpace = m_buffer[m_offset + 9U];
					}
					break;

				case 2U: {
						m_mode = m_buffer[m_offset + 0U];

						m_tx = (m_buffer[m_offset + 1U] & 0x01U) == 0x01U;
						bool adcOverflow = (m_buffer[m_offset + 1U] & 0x02U) == 0x02U;
						if (adcOverflow)
							LogError("MMDVM ADC levels have overflowed");
						bool rxOverflow = (m_buffer[m_offset + 1U] & 0x04U) == 0x04U;
						if (rxOverflow)
							LogError("MMDVM RX buffer has overflowed");
						bool txOverflow = (m_buffer[m_offset + 1U] & 0x08U) == 0x08U;
						if (txOverflow)
							LogError("MMDVM TX buffer has overflowed");
						m_lockout = (m_buffer[m_offset + 1U] & 0x10U) == 0x10U;
						bool dacOverflow = (m_buffer[m_offset + 1U] & 0x20U) == 0x20U;
						if (dacOverflow)
							LogError("MMDVM DAC levels have overflowed");
						m_cd = (m_buffer[m_offset + 1U] & 0x40U) == 0x40U;

						m_p25Space    = m_buffer[m_offset + 7U];
						m_pocsagSpace = m_buffer[m_offset + 11U];
					}
					break;

				default:
					m_p25Space    = 0U;
 					m_pocsagSpace = 0U;
					break;
				}

				m_inactivityTimer.start();
				// LogMessage("status=%02X, tx=%d, space=%u,%u,%u,%u,%u,%u,%u,%u lockout=%d, cd=%d", m_buffer[m_offset + 2U], int(m_tx), m_dstarSpace, m_dmrSpace1, m_dmrSpace2, m_ysfSpace, m_p25Space, m_nxdnSpace, m_pocsagSpace, m_fmSpace, int(m_lockout), int(m_cd));
				break;

			case MMDVM_TRANSPARENT: {
					if (m_trace)
						CUtils::dump(1U, "RX Transparent Data", m_buffer, m_length);

					unsigned char offset = m_sendTransparentDataFrameType;
					if (offset > 1U) offset = 1U;
					unsigned char data = m_length - m_offset + offset;
					m_rxTransparentData.addData(&data, 1U);

					m_rxTransparentData.addData(m_buffer + m_offset - offset, m_length - m_offset + offset);
				}
				break;

			// These should not be received, but don't complain if we do
			case MMDVM_GET_VERSION:
			case MMDVM_ACK:
				break;

			case MMDVM_NAK:
				LogWarning("Received a NAK from the MMDVM, command = 0x%02X, reason = %u", m_buffer[m_offset], m_buffer[m_offset + 1U]);
				break;

			case MMDVM_DEBUG1:
			case MMDVM_DEBUG2:
			case MMDVM_DEBUG3:
			case MMDVM_DEBUG4:
			case MMDVM_DEBUG5:
			case MMDVM_DEBUG_DUMP:
				printDebug();
				break;

			//case MMDVM_SERIAL_DATA:
			//	if (m_trace)
			//		CUtils::dump(1U, "RX Serial Data", m_buffer, m_length);
			//	m_rxSerialData.addData(m_buffer + m_offset, m_length - m_offset);
			//	break;

			// Code changed to bring the Nextion Button Pushes back to life
			case MMDVM_SERIAL_DATA:
				if (m_trace)
					CUtils::dump(1U, "RX Serial Data", m_buffer, m_length);
				
				// Original: Add to serial data buffer
				m_rxSerialData.addData(m_buffer + m_offset, m_length - m_offset);
				
				// NEW: Buffer serial data and forward complete commands as transparent data
				{
					// Add received bytes to our accumulation buffer
					for (unsigned int i = 0; i < (m_length - m_offset); i++) {
						if (m_serialDataLen < 256) {
							m_serialDataBuffer[m_serialDataLen++] = m_buffer[m_offset + i];
							
							// Check for Nextion command terminator (0xFF 0xFF 0xFF)
							if (m_serialDataLen >= 3 && 
								m_serialDataBuffer[m_serialDataLen - 3] == 0xFF &&
								m_serialDataBuffer[m_serialDataLen - 2] == 0xFF &&
								m_serialDataBuffer[m_serialDataLen - 1] == 0xFF) {
								
								// We have a complete command
								// Add it to the RX transparent data queue so it will be forwarded to NextionDriver
								// With sendFrameType=1, we need to include the frame type byte
								
								// Create a buffer with frame type byte + command data
								unsigned char frameBuffer[260];
								frameBuffer[0] = 0x90;  // Frame type: transparent data
								::memcpy(frameBuffer + 1, m_serialDataBuffer, m_serialDataLen);
								
								// Add length byte and data with frame type to RX queue
								unsigned char len = m_serialDataLen + 1U;  // +1 for frame type byte
								m_rxTransparentData.addData(&len, 1U);
								m_rxTransparentData.addData(frameBuffer, len);
								
								if (m_trace) {
									CUtils::dump(1U, "Adding button command with frame type to RX Transparent queue", frameBuffer, len);
								}
								
								// Reset buffer for next command
								m_serialDataLen = 0U;
							}
						} else {
							// Buffer overflow, reset
							LogWarning("Serial data buffer overflow, resetting");
							m_serialDataLen = 0U;
						}
					}
				}
				break;

			default:
				LogMessage("Unknown message, type: %02X", m_type);
				CUtils::dump("Buffer dump", m_buffer, m_length);
				break;
		}
	}

	// Only feed data to the modem if the playout timer has expired
	m_playoutTimer.clock(ms);
	if (!m_playoutTimer.hasExpired())
		return;

	if (m_p25Space > 1U && !m_txP25Data.isEmpty()) {
		unsigned char len = 0U;
		m_txP25Data.getData(&len, 1U);
		m_txP25Data.getData(m_buffer, len);

		if (m_trace) {
			if (m_buffer[2U] == MMDVM_P25_HDR)
				CUtils::dump(1U, "TX P25 HDR", m_buffer, len);
			else
				CUtils::dump(1U, "TX P25 LDU", m_buffer, len);
		}

		int ret = m_port->write(m_buffer, len);
		if (ret != int(len))
			LogWarning("Error when writing P25 data to the MMDVM");

		m_playoutTimer.start();

		m_p25Space--;
	}

 	if (m_pocsagSpace > 1U && !m_txPOCSAGData.isEmpty()) {
		unsigned char len = 0U;
		m_txPOCSAGData.getData(&len, 1U);
		m_txPOCSAGData.getData(m_buffer, len);

		if (m_trace)
			CUtils::dump(1U, "TX POCSAG Data", m_buffer, len);

		int ret = m_port->write(m_buffer, len);
		if (ret != int(len))
			LogWarning("Error when writing POCSAG data to the MMDVM");

		m_playoutTimer.start();

		m_pocsagSpace--;
	}

	if (!m_txTransparentData.isEmpty()) {
		unsigned char len = 0U;
		m_txTransparentData.getData(&len, 1U);
		m_txTransparentData.getData(m_buffer, len);

		if (m_trace)
			CUtils::dump(1U, "TX Transparent Data", m_buffer, len);

		int ret = m_port->write(m_buffer, len);
		if (ret != int(len))
			LogWarning("Error when writing Transparent data to the MMDVM");
	}

	if (!m_txSerialData.isEmpty()) {
		unsigned char len = 0U;
		m_txSerialData.getData(&len, 1U);
		m_txSerialData.getData(m_buffer, len);

		if (m_trace)
			CUtils::dump(1U, "TX Serial Data", m_buffer, len);

		int ret = m_port->write(m_buffer, len);
		if (ret != int(len))
			LogWarning("Error when writing Serial data to the MMDVM");
	}
}

void CModem::close()
{
	assert(m_port != nullptr);

	::LogMessage("Closing the MMDVM");

	m_port->close();
}

unsigned int CModem::readDMRData1(unsigned char* data)
{
	assert(data != nullptr);

	if (m_rxDMRData1.isEmpty())
		return 0U;

	unsigned char len = 0U;
	m_rxDMRData1.getData(&len, 1U);
	m_rxDMRData1.getData(data, len);

	return len;
}

unsigned int CModem::readDMRData2(unsigned char* data)
{
	assert(data != nullptr);

	if (m_rxDMRData2.isEmpty())
		return 0U;

	unsigned char len = 0U;
	m_rxDMRData2.getData(&len, 1U);
	m_rxDMRData2.getData(data, len);

	return len;
}

unsigned int CModem::readYSFData(unsigned char* data)
{
	assert(data != nullptr);

	if (m_rxYSFData.isEmpty())
		return 0U;

	unsigned char len = 0U;
	m_rxYSFData.getData(&len, 1U);
	m_rxYSFData.getData(data, len);

	return len;
}

unsigned int CModem::readP25Data(unsigned char* data)
{
	assert(data != nullptr);

	if (m_rxP25Data.isEmpty())
		return 0U;

	unsigned char len = 0U;
	m_rxP25Data.getData(&len, 1U);
	m_rxP25Data.getData(data, len);

	return len;
}

unsigned int CModem::readTransparentData(unsigned char* data)
{
	assert(data != nullptr);

	if (m_rxTransparentData.isEmpty())
		return 0U;

	unsigned char len = 0U;
	m_rxTransparentData.getData(&len, 1U);
	m_rxTransparentData.getData(data, len);

	return len;
}

unsigned int CModem::readSerial(unsigned char* data, unsigned int length)
{
	assert(data != nullptr);
	assert(length > 0U);

	unsigned int n = 0U;
	while (!m_rxSerialData.isEmpty() && n < length) {
		m_rxSerialData.getData(data + n, 1U);
		n++;
	}

	return n;
}

bool CModem::hasP25Space() const
{
	unsigned int space = m_txP25Data.freeSpace() / (P25_LDU_FRAME_LENGTH_BYTES + 4U);

	return space > 1U;
}

bool CModem::writeP25Data(const unsigned char* data, unsigned int length)
{
	assert(data != nullptr);
	assert(length > 0U);

	if (data[0U] != TAG_HEADER && data[0U] != TAG_DATA && data[0U] != TAG_EOT)
		return false;

	unsigned char buffer[250U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = length + 2U;
	buffer[2U] = (data[0U] == TAG_HEADER) ? MMDVM_P25_HDR : MMDVM_P25_LDU;

	::memcpy(buffer + 3U, data + 1U, length - 1U);

	unsigned char len = length + 2U;
	m_txP25Data.addData(&len, 1U);
	m_txP25Data.addData(buffer, len);

	return true;
}

bool CModem::hasPOCSAGSpace() const
{
	unsigned int space = m_txPOCSAGData.freeSpace() / (POCSAG_FRAME_LENGTH_BYTES + 4U);

	return space > 1U;
}

bool CModem::writePOCSAGData(const unsigned char* data, unsigned int length)
{
	assert(data != nullptr);
	assert(length > 0U);

	unsigned char buffer[130U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = length + 3U;
	buffer[2U] = MMDVM_POCSAG_DATA;

	::memcpy(buffer + 3U, data, length);

	unsigned char len = length + 3U;
	m_txPOCSAGData.addData(&len, 1U);
	m_txPOCSAGData.addData(buffer, len);

	return true;
}

bool CModem::writeTransparentData(const unsigned char* data, unsigned int length)
{
	assert(data != nullptr);
	assert(length > 0U);

	unsigned char buffer[250U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = length + 3U;
	buffer[2U] = MMDVM_TRANSPARENT;

	if (m_sendTransparentDataFrameType > 0U) {
		::memcpy(buffer + 2U, data, length);
		length--;
		buffer[1U]--;

		//when sendFrameType==1 , only 0x80 and 0x90 (MMDVM_SERIAL and MMDVM_TRANSPARENT) are allowed
		//  and reverted to default (MMDVM_TRANSPARENT) for any other value
		//when >1, frame type is not checked
		if (m_sendTransparentDataFrameType == 1U) {
			if ((buffer[2U] & 0xE0) != 0x80)
				buffer[2U] = MMDVM_TRANSPARENT;
		}
	} else {
		::memcpy(buffer + 3U, data, length);
	}

	unsigned char len = length + 3U;
	m_txTransparentData.addData(&len, 1U);
	m_txTransparentData.addData(buffer, len);

	return true;
}

bool CModem::writeDMRInfo(unsigned int slotNo, const std::string& src, bool group, const std::string& dest, const char* type)
{
	assert(m_port != nullptr);
	assert(type != nullptr);

	unsigned char buffer[50U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = 47U;
	buffer[2U] = MMDVM_QSO_INFO;

	buffer[3U] = MODE_DMR;

	buffer[4U] = slotNo;

	::sprintf((char*)(buffer + 5U), "%20.20s", src.c_str());

	buffer[25U] = group ? 'G' : 'I';

	::sprintf((char*)(buffer + 26U), "%20.20s", dest.c_str());

	::memcpy(buffer + 46U, type, 1U);

	return m_port->write(buffer, 47U) != 47;
}

bool CModem::writeP25Info(const char* source, bool group, unsigned int dest, const char* type)
{
	assert(m_port != nullptr);
	assert(source != nullptr);
	assert(type != nullptr);

	unsigned char buffer[40U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = 31U;
	buffer[2U] = MMDVM_QSO_INFO;

	buffer[3U] = MODE_DMR;

	::sprintf((char*)(buffer + 4U), "%20.20s", source);

	buffer[24U] = group ? 'G' : 'I';

	::sprintf((char*)(buffer + 25U), "%05u", dest);	// 16-bits

	::memcpy(buffer + 30U, type, 1U);

	return m_port->write(buffer, 31U) != 31;
}

bool CModem::writePOCSAGInfo(unsigned int ric, const std::string& message)
{
	assert(m_port != nullptr);

	size_t length = message.size();

	unsigned char buffer[250U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = (unsigned char)length + 11U;
	buffer[2U] = MMDVM_QSO_INFO;

	buffer[3U] = MODE_POCSAG;

	::sprintf((char*)(buffer + 4U), "%07u", ric);	// 21-bits

	::memcpy(buffer + 11U, message.c_str(), length);

	int ret = m_port->write(buffer, (unsigned int)length + 11U);

	return ret != int(length + 11U);
}

bool CModem::writeIPInfo(const std::string& address)
{
	assert(m_port != nullptr);

	size_t length = address.size();

	unsigned char buffer[25U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = (unsigned char)length + 4U;
	buffer[2U] = MMDVM_QSO_INFO;

	buffer[3U] = 250U;

	::memcpy(buffer + 4U, address.c_str(), length);

	int ret = m_port->write(buffer, (unsigned int)length + 4U);

	return ret != int(length + 4U);
}

bool CModem::writeSerial(const unsigned char* data, unsigned int length)
{
	assert(m_port != nullptr);
	assert(data != nullptr);
	assert(length > 0U);

	unsigned char buffer[255U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = length + 3U;
	buffer[2U] = MMDVM_SERIAL_DATA;

	::memcpy(buffer + 3U, data, length);

	unsigned char len = length + 3U;
	m_txSerialData.addData(&len, 1U);
	m_txSerialData.addData(buffer, len);

	return true;
}

bool CModem::hasTX() const
{
	return m_tx;
}

bool CModem::hasCD() const
{
	return m_cd;
}

bool CModem::hasLockout() const
{
	return m_lockout;
}

bool CModem::hasError() const
{
	return m_error;
}

bool CModem::hasDMR() const
{
	return (m_capabilities1 & CAP1_DMR) == CAP1_DMR;
}

bool CModem::hasP25() const
{
	return (m_capabilities1 & CAP1_P25) == CAP1_P25;
}

bool CModem::hasPOCSAG() const
{
	return (m_capabilities2 & CAP2_POCSAG) == CAP2_POCSAG;
}

unsigned int CModem::getVersion() const
{
	return m_protocolVersion;
}

bool CModem::readVersion()
{
	assert(m_port != nullptr);

	CThread::sleep(2000U);	// 2s

	for (unsigned int i = 0U; i < 6U; i++) {
		unsigned char buffer[3U];

		buffer[0U] = MMDVM_FRAME_START;
		buffer[1U] = 3U;
		buffer[2U] = MMDVM_GET_VERSION;

		// CUtils::dump(1U, "Written", buffer, 3U);

		int ret = m_port->write(buffer, 3U);
		if (ret != 3)
			return false;

#if defined(__APPLE__)
		m_port->setNonblock(true);
#endif

		for (unsigned int count = 0U; count < MAX_RESPONSES; count++) {
			CThread::sleep(10U);
			RESP_TYPE_MMDVM resp = getResponse();
			if ((resp == RESP_TYPE_MMDVM::OK) && (m_buffer[2U] == MMDVM_GET_VERSION)) {
				if (::memcmp(m_buffer + 4U, "MMDVM ", 6U) == 0)
					m_hwType = HW_TYPE::MMDVM;
				else if (::memcmp(m_buffer + 23U, "MMDVM ", 6U) == 0)
					m_hwType = HW_TYPE::MMDVM;
				else if (::memcmp(m_buffer + 4U, "DVMEGA", 6U) == 0)
					m_hwType = HW_TYPE::DVMEGA;
				else if (::memcmp(m_buffer + 4U, "ZUMspot", 7U) == 0)
					m_hwType = HW_TYPE::MMDVM_ZUMSPOT;
				else if (::memcmp(m_buffer + 4U, "MMDVM_HS_Hat", 12U) == 0)
					m_hwType = HW_TYPE::MMDVM_HS_HAT;
				else if (::memcmp(m_buffer + 4U, "MMDVM_HS_Dual_Hat", 17U) == 0)
					m_hwType = HW_TYPE::MMDVM_HS_DUAL_HAT;
				else if (::memcmp(m_buffer + 4U, "Nano_hotSPOT", 12U) == 0)
					m_hwType = HW_TYPE::NANO_HOTSPOT;
				else if (::memcmp(m_buffer + 4U, "Nano_DV", 7U) == 0)
					m_hwType = HW_TYPE::NANO_DV;
				else if (::memcmp(m_buffer + 4U, "D2RG_MMDVM_HS", 13U) == 0)
					m_hwType = HW_TYPE::D2RG_MMDVM_HS;
				else if (::memcmp(m_buffer + 4U, "MMDVM_HS-", 9U) == 0)
					m_hwType = HW_TYPE::MMDVM_HS;
				else if (::memcmp(m_buffer + 4U, "OpenGD77_HS", 11U) == 0)
					m_hwType = HW_TYPE::OPENGD77_HS;
				else if (::memcmp(m_buffer + 4U, "SkyBridge", 9U) == 0)
					m_hwType = HW_TYPE::SKYBRIDGE;

				m_protocolVersion = m_buffer[3U];

				switch (m_protocolVersion) {
				case 1U:
					LogInfo("MMDVM protocol version: 1, description: %.*s", m_length - 4U, m_buffer + 4U);
					m_capabilities1 = CAP1_DMR | CAP1_P25 ;
					m_capabilities2 = CAP2_POCSAG;
					return true;

				case 2U:
					LogInfo("MMDVM protocol version: 2, description: %.*s", m_length - 23U, m_buffer + 23U);
					switch (m_buffer[6U]) {
					case 0U:
						LogInfo("CPU: Atmel ARM, UDID: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", m_buffer[7U], m_buffer[8U], m_buffer[9U], m_buffer[10U], m_buffer[11U], m_buffer[12U], m_buffer[13U], m_buffer[14U], m_buffer[15U], m_buffer[16U], m_buffer[17U], m_buffer[18U], m_buffer[19U], m_buffer[20U], m_buffer[21U], m_buffer[22U]);
						break;
					case 1U:
						LogInfo("CPU: NXP ARM, UDID: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", m_buffer[7U], m_buffer[8U], m_buffer[9U], m_buffer[10U], m_buffer[11U], m_buffer[12U], m_buffer[13U], m_buffer[14U], m_buffer[15U], m_buffer[16U], m_buffer[17U], m_buffer[18U], m_buffer[19U], m_buffer[20U], m_buffer[21U], m_buffer[22U]);
						break;
					case 2U:
						LogInfo("CPU: ST-Micro ARM, UDID: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", m_buffer[7U], m_buffer[8U], m_buffer[9U], m_buffer[10U], m_buffer[11U], m_buffer[12U], m_buffer[13U], m_buffer[14U], m_buffer[15U], m_buffer[16U], m_buffer[17U], m_buffer[18U]);
						break;
					default:
						LogInfo("CPU: Unknown type: %u", m_buffer[6U]);
						break;
					}
					m_capabilities1 = m_buffer[4U];
					m_capabilities2 = m_buffer[5U];
					char modeText[100U];
					::strcpy(modeText, "Modes:");
					if (hasDMR())
						::strcat(modeText, " DMR");
					if (hasP25())
						::strcat(modeText, " P25");
					if (hasPOCSAG())
						::strcat(modeText, " POCSAG");
					LogInfo(modeText);
					return true;

				default:
					LogError("MMDVM protocol version: %u, unsupported by this version of the MMDVM Host", m_protocolVersion);
					return false;
				}

				return true;
			}
		}

		CThread::sleep(1500U);
	}

	LogError("Unable to read the firmware version after six attempts");

	return false;
}

bool CModem::readStatus()
{
	assert(m_port != nullptr);

	unsigned char buffer[3U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = 3U;
	buffer[2U] = MMDVM_GET_STATUS;

	// CUtils::dump(1U, "Written", buffer, 3U);

	return m_port->write(buffer, 3U) == 3;
}

bool CModem::writeConfig()
{
	switch (m_protocolVersion) {
	case 1U:
		return setConfig1();
	case 2U:
		return setConfig2();
	default:
		return false;
	}
}

bool CModem::setConfig1()
{
	assert(m_port != nullptr);

	unsigned char buffer[30U];

	buffer[0U] = MMDVM_FRAME_START;

	buffer[1U] = 26U;

	buffer[2U] = MMDVM_SET_CONFIG;

	buffer[3U] = 0x00U;
	if (m_rxInvert)
		buffer[3U] |= 0x01U;
	if (m_txInvert)
		buffer[3U] |= 0x02U;
	if (m_pttInvert)
		buffer[3U] |= 0x04U;
	if (m_debug)
		buffer[3U] |= 0x10U;
	if (m_useCOSAsLockout)
		buffer[3U] |= 0x20U;
	if (!m_duplex)
		buffer[3U] |= 0x80U;

	buffer[4U] = 0x00U;
	if (m_p25Enabled)
		buffer[4U] |= 0x08U;
 	if (m_pocsagEnabled)
		buffer[4U] |= 0x20U;

	buffer[5U] = m_txDelay / 10U;		// In 10ms units

	buffer[6U] = MODE_IDLE;

	buffer[7U] = (unsigned char)(m_rxLevel * 2.55F + 0.5F);

	buffer[8U] = (unsigned char)(m_cwIdTXLevel * 2.55F + 0.5F);

	buffer[11U] = 128U;           // Was OscOffset

	buffer[13U] = (unsigned char)(m_dmrTXLevel * 2.55F + 0.5F);
	buffer[15U] = (unsigned char)(m_p25TXLevel * 2.55F + 0.5F);

	buffer[16U] = (unsigned char)(m_txDCOffset + 128);
	buffer[17U] = (unsigned char)(m_rxDCOffset + 128);

 	buffer[20U] = (unsigned char)(m_pocsagTXLevel * 2.55F + 0.5F);

	buffer[22U] = (unsigned char)m_p25TXHang;

 	buffer[24U] = 0x00U;

	buffer[25U] = 0x00U;

	// CUtils::dump(1U, "Written", buffer, 26U);

	int ret = m_port->write(buffer, 26U);
	if (ret != 26)
		return false;

	unsigned int count = 0U;
	RESP_TYPE_MMDVM resp;
	do {
		CThread::sleep(10U);

		resp = getResponse();
		if ((resp == RESP_TYPE_MMDVM::OK) && (m_buffer[2U] != MMDVM_ACK) && (m_buffer[2U] != MMDVM_NAK)) {
			count++;
			if (count >= MAX_RESPONSES) {
				LogError("The MMDVM is not responding to the SET_CONFIG command");
				return false;
			}
		}
	} while ((resp == RESP_TYPE_MMDVM::OK) && (m_buffer[2U] != MMDVM_ACK) && (m_buffer[2U] != MMDVM_NAK));

	// CUtils::dump(1U, "Response", m_buffer, m_length);

	if ((resp == RESP_TYPE_MMDVM::OK) && (m_buffer[2U] == MMDVM_NAK)) {
		LogError("Received a NAK to the SET_CONFIG command from the modem");
		return false;
	}

	m_playoutTimer.start();

	return true;
}

bool CModem::setConfig2()
{
	assert(m_port != nullptr);

	unsigned char buffer[50U];

	buffer[0U] = MMDVM_FRAME_START;

	buffer[1U] = 40U;

	buffer[2U] = MMDVM_SET_CONFIG;

	buffer[3U] = 0x00U;
	if (m_rxInvert)
		buffer[3U] |= 0x01U;
	if (m_txInvert)
		buffer[3U] |= 0x02U;
	if (m_pttInvert)
		buffer[3U] |= 0x04U;
	if (m_debug)
		buffer[3U] |= 0x10U;
	if (m_useCOSAsLockout)
		buffer[3U] |= 0x20U;
	if (!m_duplex)
		buffer[3U] |= 0x80U;

	buffer[4U] = 0x00U;
	if (m_p25Enabled)
		buffer[4U] |= 0x08U;

	buffer[5U] = 0x00U;
	if (m_pocsagEnabled)
		buffer[5U] |= 0x01U;

	buffer[6U] = m_txDelay / 10U;		// In 10ms units

	buffer[7U] = MODE_IDLE;

	buffer[8U] = (unsigned char)(m_txDCOffset + 128);
	buffer[9U] = (unsigned char)(m_rxDCOffset + 128);

	buffer[10U] = (unsigned char)(m_rxLevel * 2.55F + 0.5F);

	buffer[11U] = (unsigned char)(m_cwIdTXLevel * 2.55F + 0.5F);
	buffer[13U] = (unsigned char)(m_dmrTXLevel * 2.55F + 0.5F);
	buffer[15U] = (unsigned char)(m_p25TXLevel * 2.55F + 0.5F);
 	buffer[17U] = 0x00U;
	buffer[18U] = (unsigned char)(m_pocsagTXLevel * 2.55F + 0.5F);
	buffer[20U] = 0x00U;
	buffer[21U] = 0x00U;
	buffer[22U] = 0x00U;

	buffer[24U] = (unsigned char)m_p25TXHang;
 	buffer[26U] = 0x00U;
	buffer[27U] = 0x00U;
	buffer[28U] = 0x00U;

	buffer[31U] = 128U;
	buffer[32U] = 0x00U;
	buffer[33U] = 0x00U;
	buffer[34U] = 0x00U;

	buffer[35U] = 0x00U;
	buffer[36U] = 0x00U;
	buffer[37U] = 0x00U;
	buffer[38U] = 0x00U;
	buffer[39U] = 0x00U;

	// CUtils::dump(1U, "Written", buffer, 40U);

	int ret = m_port->write(buffer, 40U);
	if (ret != 40)
		return false;

	unsigned int count = 0U;
	RESP_TYPE_MMDVM resp;
	do {
		CThread::sleep(10U);

		resp = getResponse();
		if ((resp == RESP_TYPE_MMDVM::OK) && (m_buffer[2U] != MMDVM_ACK) && (m_buffer[2U] != MMDVM_NAK)) {
			count++;
			if (count >= MAX_RESPONSES) {
				LogError("The MMDVM is not responding to the SET_CONFIG command");
				return false;
			}
		}
	} while ((resp == RESP_TYPE_MMDVM::OK) && (m_buffer[2U] != MMDVM_ACK) && (m_buffer[2U] != MMDVM_NAK));

	// CUtils::dump(1U, "Response", m_buffer, m_length);

	if ((resp == RESP_TYPE_MMDVM::OK) && (m_buffer[2U] == MMDVM_NAK)) {
		LogError("Received a NAK to the SET_CONFIG command from the modem");
		return false;
	}

	m_playoutTimer.start();

	return true;
}

bool CModem::setFrequency()
{
	assert(m_port != nullptr);

	unsigned char buffer[20U];
	unsigned char len;
	unsigned int  pocsagFrequency = 433000000U;

	if (m_pocsagEnabled)
		pocsagFrequency = m_pocsagFrequency;

	if (m_hwType == HW_TYPE::DVMEGA)
		len = 12U;
	else {
		buffer[12U]  = (unsigned char)(m_rfLevel * 2.55F + 0.5F);

		buffer[13U] = (pocsagFrequency >> 0)  & 0xFFU;
		buffer[14U] = (pocsagFrequency >> 8)  & 0xFFU;
		buffer[15U] = (pocsagFrequency >> 16) & 0xFFU;
		buffer[16U] = (pocsagFrequency >> 24) & 0xFFU;

		len = 17U;
	}

	buffer[0U]  = MMDVM_FRAME_START;

	buffer[1U]  = len;

	buffer[2U]  = MMDVM_SET_FREQ;

	buffer[3U]  = 0x00U;

	buffer[4U]  = (m_rxFrequency >> 0) & 0xFFU;
	buffer[5U]  = (m_rxFrequency >> 8) & 0xFFU;
	buffer[6U]  = (m_rxFrequency >> 16) & 0xFFU;
	buffer[7U]  = (m_rxFrequency >> 24) & 0xFFU;

	buffer[8U]  = (m_txFrequency >> 0) & 0xFFU;
	buffer[9U]  = (m_txFrequency >> 8) & 0xFFU;
	buffer[10U] = (m_txFrequency >> 16) & 0xFFU;
	buffer[11U] = (m_txFrequency >> 24) & 0xFFU;

	// CUtils::dump(1U, "Written", buffer, len);

	int ret = m_port->write(buffer, len);
	if (ret != len)
		return false;

	unsigned int count = 0U;
	RESP_TYPE_MMDVM resp;
	do {
		CThread::sleep(10U);

		resp = getResponse();
		if ((resp == RESP_TYPE_MMDVM::OK) && (m_buffer[2U] != MMDVM_ACK) && (m_buffer[2U] != MMDVM_NAK)) {
			count++;
			if (count >= MAX_RESPONSES) {
				LogError("The MMDVM is not responding to the SET_FREQ command");
				return false;
			}
		}
	} while ((resp == RESP_TYPE_MMDVM::OK) && (m_buffer[2U] != MMDVM_ACK) && (m_buffer[2U] != MMDVM_NAK));

	// CUtils::dump(1U, "Response", m_buffer, m_length);

	if ((resp == RESP_TYPE_MMDVM::OK) && (m_buffer[2U] == MMDVM_NAK)) {
		LogError("Received a NAK to the SET_FREQ command from the modem");
		return false;
	}

	return true;
}

RESP_TYPE_MMDVM CModem::getResponse()
{
	assert(m_port != nullptr);

	if (m_state == SERIAL_STATE::START) {
		// Get the start of the frame or nothing at all
		int ret = m_port->read(m_buffer + 0U, 1U);
		if (ret < 0) {
			LogError("Error when reading from the modem");
			return RESP_TYPE_MMDVM::ERR;
		}

		if (ret == 0)
			return RESP_TYPE_MMDVM::TIMEOUT;

		if (m_buffer[0U] != MMDVM_FRAME_START)
			return RESP_TYPE_MMDVM::TIMEOUT;

		m_state  = SERIAL_STATE::LENGTH1;
		m_length = 1U;
	}

	if (m_state == SERIAL_STATE::LENGTH1) {
		// Get the length of the frame, 1/2
		int ret = m_port->read(m_buffer + 1U, 1U);
		if (ret < 0) {
			LogError("Error when reading from the modem");
			m_state = SERIAL_STATE::START;
			return RESP_TYPE_MMDVM::ERR;
		}

		if (ret == 0)
			return RESP_TYPE_MMDVM::TIMEOUT;

		m_length = m_buffer[1U];
		m_offset = 2U;

		if (m_length == 0U)
			m_state = SERIAL_STATE::LENGTH2;
		else
			m_state = SERIAL_STATE::TYPE;
	}

	if (m_state == SERIAL_STATE::LENGTH2) {
		// Get the length of the frane, 2/2
		int ret = m_port->read(m_buffer + 2U, 1U);
		if (ret < 0) {
			LogError("Error when reading from the modem");
			m_state = SERIAL_STATE::START;
			return RESP_TYPE_MMDVM::ERR;
		}

		if (ret == 0)
			return RESP_TYPE_MMDVM::TIMEOUT;

		m_length = m_buffer[2U] + 255U;
		m_offset = 3U;
		m_state  = SERIAL_STATE::TYPE;
	}

	if (m_state == SERIAL_STATE::TYPE) {
		// Get the frame type
		int ret = m_port->read(&m_type, 1U);
		if (ret < 0) {
			LogError("Error when reading from the modem");
			m_state = SERIAL_STATE::START;
			return RESP_TYPE_MMDVM::ERR;
		}

		if (ret == 0)
			return RESP_TYPE_MMDVM::TIMEOUT;

		m_buffer[m_offset++] = m_type;

		m_state = SERIAL_STATE::DATA;
	}

	if (m_state == SERIAL_STATE::DATA) {
		while (m_offset < m_length) {
			int ret = m_port->read(m_buffer + m_offset, m_length - m_offset);
			if (ret < 0) {
				LogError("Error when reading from the modem");
				m_state = SERIAL_STATE::START;
				return RESP_TYPE_MMDVM::ERR;
			}

			if (ret == 0)
				return RESP_TYPE_MMDVM::TIMEOUT;

			if (ret > 0)
				m_offset += ret;
		}
	}

	// CUtils::dump(1U, "Received", m_buffer, m_length);

	m_offset = m_length > 255U ? 4U : 3U;
	m_state  = SERIAL_STATE::START;

	return RESP_TYPE_MMDVM::OK;
}

HW_TYPE CModem::getHWType() const
{
	return m_hwType;
}

unsigned char CModem::getMode() const
{
	return m_mode;
}

bool CModem::setMode(unsigned char mode)
{
	assert(m_port != nullptr);

	unsigned char buffer[4U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = 4U;
	buffer[2U] = MMDVM_SET_MODE;
	buffer[3U] = mode;

	// CUtils::dump(1U, "Written", buffer, 4U);

	return m_port->write(buffer, 4U) == 4;
}

bool CModem::sendCWId(const std::string& callsign)
{
	assert(m_port != nullptr);

	unsigned int length = (unsigned int)callsign.length();
	if (length > 200U)
		length = 200U;

	unsigned char buffer[205U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = length + 3U;
	buffer[2U] = MMDVM_SEND_CWID;

	for (unsigned int i = 0U; i < length; i++)
		buffer[i + 3U] = callsign.at(i);

	// CUtils::dump(1U, "Written", buffer, length + 3U);

	return m_port->write(buffer, length + 3U) == int(length + 3U);
}

bool CModem::writeDMRStart(bool tx)
{
	assert(m_port != nullptr);

	if (tx && m_tx)
		return true;
	if (!tx && !m_tx)
		return true;

	unsigned char buffer[4U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = 4U;
	buffer[2U] = MMDVM_DMR_START;
	buffer[3U] = tx ? 0x01U : 0x00U;

	// CUtils::dump(1U, "Written", buffer, 4U);

	return m_port->write(buffer, 4U) == 4;
}

bool CModem::writeDMRAbort(unsigned int slotNo)
{
	assert(m_port != nullptr);

	if (slotNo == 1U)
		m_txDMRData1.clear();
	else
		m_txDMRData2.clear();

	unsigned char buffer[4U];

	buffer[0U] = MMDVM_FRAME_START;
	buffer[1U] = 4U;
	buffer[2U] = MMDVM_DMR_ABORT;
	buffer[3U] = slotNo;

	// CUtils::dump(1U, "Written", buffer, 4U);

	return m_port->write(buffer, 4U) == 4;
}

bool CModem::writeDMRShortLC(const unsigned char* lc)
{
	assert(m_port != nullptr);
	assert(lc != nullptr);

	unsigned char buffer[12U];

	buffer[0U]  = MMDVM_FRAME_START;
	buffer[1U]  = 12U;
	buffer[2U]  = MMDVM_DMR_SHORTLC;
	buffer[3U]  = lc[0U];
	buffer[4U]  = lc[1U];
	buffer[5U]  = lc[2U];
	buffer[6U]  = lc[3U];
	buffer[7U]  = lc[4U];
	buffer[8U]  = lc[5U];
	buffer[9U]  = lc[6U];
	buffer[10U] = lc[7U];
	buffer[11U] = lc[8U];

	// CUtils::dump(1U, "Written", buffer, 12U);

	return m_port->write(buffer, 12U) == 12;
}

 void CModem::printDebug()
{
	if (m_type == MMDVM_DEBUG1) {
		LogMessage("Debug: %.*s", m_length - m_offset - 0U, m_buffer + m_offset);
	} else if (m_type == MMDVM_DEBUG2) {
		short val1 = (m_buffer[m_length - 2U] << 8) | m_buffer[m_length - 1U];
		LogMessage("Debug: %.*s %d", m_length - m_offset - 2U, m_buffer + m_offset, val1);
	} else if (m_type == MMDVM_DEBUG3) {
		short val1 = (m_buffer[m_length - 4U] << 8) | m_buffer[m_length - 3U];
		short val2 = (m_buffer[m_length - 2U] << 8) | m_buffer[m_length - 1U];
		LogMessage("Debug: %.*s %d %d", m_length - m_offset - 4U, m_buffer + m_offset, val1, val2);
	} else if (m_type == MMDVM_DEBUG4) {
		short val1 = (m_buffer[m_length - 6U] << 8) | m_buffer[m_length - 5U];
		short val2 = (m_buffer[m_length - 4U] << 8) | m_buffer[m_length - 3U];
		short val3 = (m_buffer[m_length - 2U] << 8) | m_buffer[m_length - 1U];
		LogMessage("Debug: %.*s %d %d %d", m_length - m_offset - 6U, m_buffer + m_offset, val1, val2, val3);
	} else if (m_type == MMDVM_DEBUG5) {
		short val1 = (m_buffer[m_length - 8U] << 8) | m_buffer[m_length - 7U];
		short val2 = (m_buffer[m_length - 6U] << 8) | m_buffer[m_length - 5U];
		short val3 = (m_buffer[m_length - 4U] << 8) | m_buffer[m_length - 3U];
		short val4 = (m_buffer[m_length - 2U] << 8) | m_buffer[m_length - 1U];
		LogMessage("Debug: %.*s %d %d %d %d", m_length - m_offset - 8U, m_buffer + m_offset, val1, val2, val3, val4);
	} else if (m_type == MMDVM_DEBUG_DUMP) {
		CUtils::dump(1U, "Debug: Data", m_buffer + m_offset, m_length - m_offset);
	}
}
