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

#ifndef	Modem_H
#define	Modem_H

#include "ModemPort.h"
#include "RingBuffer.h"
#include "Defines.h"
#include "Timer.h"

#include <string>

enum class RESP_TYPE_MMDVM {
	OK,
	TIMEOUT,
	ERR
};

enum class SERIAL_STATE {
	START,
	LENGTH1,
	LENGTH2,
	TYPE,
	DATA
};

class CModem {
public:
	CModem(bool duplex, bool rxInvert, bool txInvert, bool pttInvert, unsigned int txDelay, unsigned int dmrDelay, bool useCOSAsLockout, bool trace, bool debug);
	~CModem();

	void setPort(IModemPort* port);
	void setRFParams(unsigned int rxFrequency, int rxOffset, unsigned int txFrequency, int txOffset, int txDCOffset, int rxDCOffset, float rfLevel, unsigned int pocsagFrequency);
	void setModeParams(bool p25Enabled, bool pocsagEnabled);
	void setLevels(float rxLevel, float cwIdTXLevel, float dmrTXLevel, float p25TXLevel, float pocsagLevel);
	void setYSFParams(bool loDev, unsigned int txHang);
	void setP25Params(unsigned int txHang);
	void setTransparentDataParams(unsigned int sendFrameType);

	bool open();

	bool hasDMR() const;
	bool hasP25() const;
	bool hasPOCSAG() const;

	unsigned int getVersion() const;

	unsigned int readDMRData1(unsigned char* data);
	unsigned int readDMRData2(unsigned char* data);
	unsigned int readYSFData(unsigned char* data);
	unsigned int readP25Data(unsigned char* data);

	bool hasDMRSpace1() const;
	bool hasDMRSpace2() const;
	bool hasYSFSpace() const;
	bool hasP25Space() const;
	bool hasPOCSAGSpace() const;

	bool hasTX() const;
	bool hasCD() const;

	bool hasLockout() const;
	bool hasError() const;

	bool writeConfig();
	bool writeDMRData1(const unsigned char* data, unsigned int length);
	bool writeDMRData2(const unsigned char* data, unsigned int length);
	bool writeYSFData(const unsigned char* data, unsigned int length);
	bool writeP25Data(const unsigned char* data, unsigned int length);
	bool writePOCSAGData(const unsigned char* data, unsigned int length);

	bool writeDMRInfo(unsigned int slotNo, const std::string& src, bool group, const std::string& dst, const char* type);
	bool writeYSFInfo(const char* source, const char* dest, unsigned char dgid, const char* type, const char* origin);
	bool writeP25Info(const char* source, bool group, unsigned int dest, const char* type);
	bool writePOCSAGInfo(unsigned int ric, const std::string& message);
	bool writeIPInfo(const std::string& address);

	bool writeDMRStart(bool tx);
	bool writeDMRShortLC(const unsigned char* lc);
	bool writeDMRAbort(unsigned int slotNo);

	bool writeTransparentData(const unsigned char* data, unsigned int length);
	unsigned int readTransparentData(unsigned char* data);

	bool writeSerial(const unsigned char* data, unsigned int length);
	unsigned int readSerial(unsigned char* data, unsigned int length);

	unsigned char getMode() const;
	bool setMode(unsigned char mode);

	bool sendCWId(const std::string& callsign);

	HW_TYPE getHWType() const;

	void clock(unsigned int ms);

	void close();

private:
	unsigned int               m_protocolVersion;
	unsigned int               m_p25TXHang;
	bool                       m_duplex;
	bool                       m_rxInvert;
	bool                       m_txInvert;
	bool                       m_pttInvert;
	unsigned int               m_txDelay;
	float                      m_rxLevel;
	float                      m_cwIdTXLevel;
	float                      m_dmrTXLevel;
	float                      m_p25TXLevel;
	float                      m_pocsagTXLevel;
	float                      m_rfLevel;
	bool                       m_useCOSAsLockout;
	bool                       m_trace;
	bool                       m_debug;
	unsigned int               m_rxFrequency;
	unsigned int               m_txFrequency;
	unsigned int               m_pocsagFrequency;
	bool                       m_p25Enabled;
	bool                       m_pocsagEnabled;
	int                        m_rxDCOffset;
	int                        m_txDCOffset;
	IModemPort*                m_port;
	unsigned char*             m_buffer;
	unsigned int               m_length;
	unsigned int               m_offset;
	SERIAL_STATE               m_state;
	unsigned char              m_type;
	CRingBuffer<unsigned char> m_rxDMRData1;
	CRingBuffer<unsigned char> m_rxDMRData2;
	CRingBuffer<unsigned char> m_txDMRData1;
	CRingBuffer<unsigned char> m_txDMRData2;
	CRingBuffer<unsigned char> m_rxYSFData;
	CRingBuffer<unsigned char> m_txYSFData;
	CRingBuffer<unsigned char> m_rxP25Data;
	CRingBuffer<unsigned char> m_txP25Data;
	CRingBuffer<unsigned char> m_txPOCSAGData;
	CRingBuffer<unsigned char> m_rxSerialData;
	CRingBuffer<unsigned char> m_txSerialData;
	CRingBuffer<unsigned char> m_rxTransparentData;
	CRingBuffer<unsigned char> m_txTransparentData;
	unsigned int               m_sendTransparentDataFrameType;
	CTimer                     m_statusTimer;
	CTimer                     m_inactivityTimer;
	CTimer                     m_playoutTimer;
	unsigned int               m_p25Space;
	unsigned int               m_pocsagSpace;
	bool                       m_tx;
	bool                       m_cd;
	bool                       m_lockout;
	bool                       m_error;
	unsigned char              m_mode;
	HW_TYPE                    m_hwType;

 	unsigned char              m_capabilities1;
	unsigned char              m_capabilities2;

	bool readVersion();
	bool readStatus();
	bool setConfig1();
	bool setConfig2();
	bool setFrequency();

	void printDebug();

	RESP_TYPE_MMDVM getResponse();

	// Added these for buffering serial data from display:
    unsigned char              m_serialDataBuffer[256];
    unsigned int               m_serialDataLen;
};

#endif
