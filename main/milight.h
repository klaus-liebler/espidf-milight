//Various snippets from
//https://github.com/nopnop2002/esp-idf-mirf
//https://arduino-projects4u.com/milight-new-protocol/


#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#include "nrf24_registers.h"

#define TAG "NRF24"



constexpr char rf24_datarates[][8] = {"1MBPS", "2MBPS", "250KBPS"};
constexpr char rf24_crclength[][10] = {"Disabled", "8 bits", "16 bits"};
constexpr char rf24_pa_dbm[][8] = {"PA_MIN", "PA_LOW", "PA_HIGH", "PA_MAX"};




class Nrf24Receiver
{
private:
	uint8_t PTX;		//In sending mode.
	gpio_num_t cePin;	// CE Pin controls RX / TX
	uint8_t payloadLen; // Payload width in bytes
	spi_device_handle_t _SPIHandle;
	uint8_t buf16[16] __attribute__((aligned(4)));

	void Nrf24_configRegister(uint8_t reg, uint8_t value)
	{
		buf16[0] = (W_REGISTER | (REGISTER_MASK & reg));
		buf16[1] = value;
		spiTransaction(buf16, 2);
	}

	void singleByteCommand(uint8_t cmd)
	{
		buf16[0] = cmd;
		spiTransaction(buf16, 1);
	}

	void Nrf24_writeRegistersStartingWith1inBuf(uint8_t reg, size_t len)
	{
		buf16[0] = (W_REGISTER | (REGISTER_MASK & reg));
		spiTransaction(buf16, 1 + len);
	}

	uint8_t readRegister(uint8_t reg)
	{
		buf16[0] = (R_REGISTER | (REGISTER_MASK & reg));
		spiTransaction(buf16, 2);
		return buf16[1];
	}

	void Nrf24_readRegisters(uint8_t reg, uint8_t len)
	{
		buf16[0] = (R_REGISTER | (REGISTER_MASK & reg));
		spiTransaction(buf16, 1 + len);
	}

	void Nrf24_ceHi()
	{
		gpio_set_level(cePin, 1);
	}

	void Nrf24_ceLow()
	{
		gpio_set_level(cePin, 0);
	}

public:
	void setup(spi_host_device_t hostDevice, int dmaChannel, gpio_num_t ce_pin, gpio_num_t csn_pin, gpio_num_t miso_pin, gpio_num_t mosi_pin, gpio_num_t sclk_pin)
	{
		gpio_pad_select_gpio(ce_pin);
		gpio_set_direction(ce_pin, GPIO_MODE_OUTPUT);
		gpio_set_level(ce_pin, 0);

		spi_bus_config_t spi_bus_config{};

		spi_bus_config.sclk_io_num = sclk_pin;
		spi_bus_config.mosi_io_num = mosi_pin;
		spi_bus_config.miso_io_num = miso_pin;
		spi_bus_config.quadwp_io_num = GPIO_NUM_NC;
		spi_bus_config.quadhd_io_num = GPIO_NUM_NC;

		ESP_ERROR_CHECK(spi_bus_initialize(hostDevice, &spi_bus_config, dmaChannel));

		spi_device_interface_config_t devcfg{};
		devcfg.clock_speed_hz = SPI_MASTER_FREQ_8M;
		devcfg.queue_size = 1;
		devcfg.mode = 0;
		devcfg.flags = 0;
		devcfg.spics_io_num = csn_pin;

		spi_device_handle_t handle;
		ESP_ERROR_CHECK(spi_bus_add_device(hostDevice, &devcfg, &handle));

		cePin = ce_pin;
		_SPIHandle = handle;
		payloadLen = 14;
	}

	void spiTransaction(uint8_t *buf, size_t len)
	{
		assert(((int)buf & 0b11) == 0);
		spi_transaction_t SPITransaction{};
		SPITransaction.length = len * 8;
		SPITransaction.tx_buffer = buf;
		SPITransaction.rx_buffer = buf;
		spi_device_transmit(_SPIHandle, &SPITransaction);
	}

	//Set tx power : 0=-18dBm,1=-12dBm,2=-6dBm,3=0dBm,
	//Select between the high speed data rates:0=1Mbps, 1=2Mbps, 2=250Kbps
	void config(uint8_t channel, uint8_t payloadLen, const uint8_t *const readAddr, uint8_t readAddrLen, uint8_t en_aa, Rf24Datarate speed, Rf24PowerAmp txPower)

	// Sets the important registers in the MiRF module and powers the module
	// in receiving mode
	// NB: channel and payload must be set now.
	{
		this->payloadLen = payloadLen;

		Nrf24_ceLow();

		memcpy(buf16 + 1, readAddr, readAddrLen);
		Nrf24_writeRegistersStartingWith1inBuf(RX_ADDR_P1, readAddrLen);
		uint8_t val = 0b10;
		Nrf24_configRegister(EN_RXADDR, val);
		Nrf24_ceHi();

		if ((int)speed > 1)
		{
			Nrf24_configRegister(RF_SETUP, (1 << RF_DR_LOW));
		}
		else
		{
			Nrf24_configRegister(RF_SETUP, (((int)speed) << RF_DR_HIGH));
		}

		Nrf24_configRegister(EN_AA, en_aa);
		Nrf24_configRegister(RF_SETUP, ((int)txPower) << RF_PWR);
		Nrf24_configRegister(RF_CH, channel);		// Set RF channel
		Nrf24_configRegister(RX_PW_P0, payloadLen); // Set length of incoming payload
		Nrf24_configRegister(RX_PW_P1, payloadLen);
		Nrf24_powerUpRx(); // Start receiver
		Nrf24_flushRx();
	}

	bool IsDataReady() // Checks if data is available for reading
	{
		uint8_t status = getStatus(); // See note in getData() function - just checking RX_DR isn't good enough
		if (status & (1 << RX_DR))
			return 1;			 // We can short circuit on RX_DR, but if it's not set, we still need
		return !IsRxFifoEmpty(); // to check the FIFO for any pending packets
	}

	bool IsRxFifoEmpty()
	{
		uint8_t fifoStatus = readRegister(FIFO_STATUS);
		return (fifoStatus & (1 << RX_EMPTY));
	}

	/*
First returned byte is status. data buffer must have a length of min PAYLOAD_LEN+1
*/
	void GetRxData(uint8_t *data) // Reads payload bytes into data array
	{
		data[0] = R_RX_PAYLOAD;
		spiTransaction(data, payloadLen + 1);
		// Pull up chip select
		// NVI: per product spec, p 67, note c:
		//	"The RX_DR IRQ is asserted by a new packet arrival event. The procedure
		//	for handling this interrupt should be: 1) read payload through SPI,
		//	2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more
		//	payloads available in RX FIFO, 4) if there are more data in RX FIFO,
		//	repeat from step 1)."
		// So if we're going to clear RX_DR here, we need to check the RX FIFO
		// in the dataReady() function
		Nrf24_configRegister(STATUS, (1 << RX_DR)); // Reset status register
	}

	uint8_t getStatus()
	{
		return readRegister(STATUS);
	}

	void Nrf24_powerUpRx()
	{
		PTX = 0;
		Nrf24_ceLow();
		Nrf24_configRegister(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (1 << PRIM_RX))); //set device as RX mode
		Nrf24_ceHi();
		Nrf24_configRegister(STATUS, (1 << TX_DS) | (1 << MAX_RT)); //Clear seeded interrupt and max tx number interrupt
	}

	void Nrf24_flushRx()
	{
		singleByteCommand(FLUSH_RX);
	}

	void Nrf24_powerDown()
	{
		Nrf24_ceLow();
		Nrf24_configRegister(CONFIG, mirf_CONFIG);
	}

#define _BV(x) (1 << (x))

	void printDetails()
	{

		printf("================ SPI Configuration ================\n");
		printf("CE Pin	\t = GPIO%d\n", cePin);
		printf("================ NRF Configuration ================\n");

		Nrf24_print_status(getStatus());
		Nrf24_print_byte_register("CONFIG\t", CONFIG, 1);
		Nrf24_print_byte_register("EN_AA\t", EN_AA, 1);
		Nrf24_print_byte_register("EN_RXADDR", EN_RXADDR, 1);
		Nrf24_print_byte_register("SETUP_ADDRW", SETUP_AW, 1);
		Nrf24_print_byte_register("RF_CH\t", RF_CH, 1);
		Nrf24_print_byte_register("RF_SETUP", RF_SETUP, 1);

		Nrf24_print_address_register("RX_ADDR_P0-1", RX_ADDR_P0, 2);
		Nrf24_print_byte_register("RX_ADDR_P2-5", RX_ADDR_P2, 4);
		Nrf24_print_address_register("TX_ADDR\t", TX_ADDR, 1);
		Nrf24_print_byte_register("RX_PW_P0-6", RX_PW_P0, 6);
		Nrf24_print_byte_register("DYNPD/FEATURE", DYNPD, 2);
		//printf("getDataRate()=%d\n",Nrf24_getDataRate(dev));
		printf("Data Rate\t = %s\n", rf24_datarates[(int)Nrf24_getDataRate()]);
#if 0
	printf_P(PSTR("Model\t\t = "
	PRIPSTR
	"\r\n"),pgm_read_ptr(&rf24_model_e_str_P[isPVariant()]));
#endif
		//printf("getCRCLength()=%d\n",Nrf24_getCRCLength(dev));
		printf("CRC Length\t = %s\n", rf24_crclength[(int)Nrf24_getCRCLength()]);
		//printf("getPALevel()=%d\n",Nrf24_getPALevel());
		printf("PA Power\t = %s\n", rf24_pa_dbm[Nrf24_getPALevel()]);
	}

	void Nrf24_print_status(uint8_t status)
	{
		printf("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n", status, (status & _BV(RX_DR)) ? 1 : 0,
			   (status & _BV(TX_DS)) ? 1 : 0, (status & _BV(MAX_RT)) ? 1 : 0, ((status >> RX_P_NO) & 0x07), (status & _BV(TX_FULL)) ? 1 : 0);
	}

	void Nrf24_print_address_register(const char *name, uint8_t reg, uint8_t qty)
	{
		printf("%s\t =", name);
		while (qty--)
		{
			Nrf24_readRegisters(reg++, 5);
			uint8_t *buffer = buf16 + 1;
			printf(" 0x");
			uint8_t *bufptr = buffer + 5;
			while (--bufptr >= buffer)
			{
				printf("%02x", *bufptr);
			}
		}
		printf("\r\n");
	}

	void Nrf24_print_byte_register(const char *name, uint8_t reg, uint8_t qty)
	{
		printf("%s\t =", name);
		while (qty--)
		{
			uint8_t buffer = readRegister(reg++);
			printf(" 0x%02x", buffer);
		}
		printf("\r\n");
	}

	Rf24Datarate Nrf24_getDataRate()
	{
		Rf24Datarate result;
		uint8_t dr = readRegister(RF_SETUP);
		dr = dr & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

		// switch uses RAM (evil!)
		// Order matters in our case below
		if (dr == _BV(RF_DR_LOW))
		{
			// '10' = 250KBPS
			result = Rf24Datarate::RF24_250KBPS;
		}
		else if (dr == _BV(RF_DR_HIGH))
		{
			// '01' = 2MBPS
			result = Rf24Datarate::RF24_2MBPS;
		}
		else
		{
			// '00' = 1MBPS
			result = Rf24Datarate::RF24_1MBPS;
		}
		return result;
	}

	rf24_crclength_e Nrf24_getCRCLength()
	{
		rf24_crclength_e result = rf24_crclength_e::RF24_CRC_DISABLED;

		uint8_t config = readRegister(CONFIG);
		config = config & (_BV(CRCO) | _BV(EN_CRC));
		uint8_t AA = readRegister(EN_AA);

		if (config & _BV(EN_CRC) || AA)
		{
			if (config & _BV(CRCO))
			{
				result = rf24_crclength_e::RF24_CRC_16;
			}
			else
			{
				result = rf24_crclength_e::RF24_CRC_8;
			}
		}

		return result;
	}

	uint8_t Nrf24_getPALevel()
	{
		uint8_t level = readRegister(RF_SETUP);
		level = (level & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
		return (level);
	}
};


class MilightDecoder
{

private:
	uint8_t V2_OFFSET(uint8_t byte, uint8_t key, uint8_t jumpStart)
	{
		static const uint8_t V2_OFFSETS[][4]{
			{0x45, 0x1F, 0x14, 0x5C},
			{0x2B, 0xC9, 0xE3, 0x11},
			{0xEE, 0xDE, 0x0B, 0xAA},
			{0xAF, 0x03, 0x1D, 0xF3},
			{0x1A, 0xE2, 0xF0, 0xD1},
			{0x04, 0xD8, 0x71, 0x42},
			{0xAF, 0x04, 0xDD, 0x07},
			{0xE1, 0x93, 0xB8, 0xE4}};
		return V2_OFFSETS[byte - 1][key % 4] + ((jumpStart > 0 && key >= jumpStart && key <= jumpStart + 0x80) ? 0x80 : 0);
	}

	const uint8_t V2_OFFSET_JUMP_START = 0x54;

	uint8_t xorKey(uint8_t key)
	{
		// Generate most significant nibble
		const uint8_t shift = (key & 0x0F) < 0x04 ? 0 : 1;
		const uint8_t x = (((key & 0xF0) >> 4) + shift + 6) % 8;
		const uint8_t msn = (((4 + x) ^ 1) & 0x0F) << 4;
		// Generate least significant nibble
		const uint8_t lsn = ((((key & 0x0F) + 4) ^ 2) & 0x0F);
		return (msn | lsn);
	}
	uint8_t decodeByte(uint8_t byte, uint8_t s1, uint8_t xorKey, uint8_t s2)
	{
		uint8_t value = byte - s2;
		value = value ^ xorKey;
		value = value - s1;
		return value;
	}

	uint16_t calc_crc(uint8_t *data, size_t data_length)
	{
		static const uint16_t CRC_POLY = 0x8408;
		uint16_t state = 0;
		for (size_t i = 0; i < data_length; i++)
		{
			uint8_t byte = data[i];
			for (int j = 0; j < 8; j++)
			{
				if ((byte ^ state) & 0x01)
				{
					state = (state >> 1) ^ CRC_POLY;
				}
				else
				{
					state = state >> 1;
				}
				byte = byte >> 1;
			}
		}
		return state;
	}

	

	uint8_t getByte(uint8_t *src, size_t i)
	{ //i=0 meint:length
		uint8_t highNibble = rev[src[i + 2] >> 4];
		uint8_t lowNibble = rev[src[i + 1] & 0x0F];
		return (highNibble << 4) | (lowNibble & 0x0F);
	}

	int transformPacket(uint8_t *tmp)
	{
		static const uint8_t LAST_SYNC_BYTE = 0x18; //for 9-Byte payload
		if (tmp[0] != LAST_SYNC_BYTE)
			return -1;
		if ((tmp[1] & 0xF0) != 0xA0)
			return -2;
		if ((tmp[1] & 0xF0) != 0xA0)
			return -3;
		int len = tmp[0] = getByte(tmp, 0);	  //get payload length
		for (int i = 1; i < 1 + len + 2; i++) //length item + playload length + crc
		{
			tmp[i] = getByte(tmp, i);
		}
		uint16_t crcShouldbe = calc_crc(tmp, len + 1);
		uint16_t crcIs = (tmp[len + 2] << 8) | tmp[len + 1];

		if (crcShouldbe != crcIs)
		{
			//ESP_LOGE(TAG, "CRC ERROR crcShouldbe=0x%04X, crcIs=0x%04X", crcShouldbe, crcIs);
			return -4;
		}
		return 0;
	}

	uint32_t GetPacketId(uint8_t *packet)
	{
		return (((packet[1] & 0xF0) << 24) | (packet[2] << 16) | (packet[3] << 8) | (packet[7]));
	}

	Nrf24Receiver *recv;
	uint8_t buf[20] __attribute__((aligned(4)));
	uint32_t previousID = 0;

public:
	MilightDecoder(Nrf24Receiver *recv) : recv(recv)
	{
	}

	bool TryReceiveNewPacket(uint8_t *cmd, uint8_t *arg)
	{
		if (!recv->IsDataReady())
			return false;
		recv->GetRxData(buf);
		uint8_t *packet = buf + 1; //to ignore the very first byte, which is the satus byte (clocked out first). packet points to the packetLen
		//ESP_LOG_BUFFER_HEXDUMP("RAW", packet, PAYLOAD_SIZE, ESP_LOG_INFO);
		int err = transformPacket(packet);
		if (err < 0)
		{
			return false;
		}
		//ESP_LOG_BUFFER_HEXDUMP("TRANS", packet, 9, ESP_LOG_INFO);
		uint32_t id = GetPacketId(packet);
		packet = buf + 2; //packet points to the payload
		uint8_t key = xorKey(packet[0]);
		for (size_t i = 1; i <= 7; i++)
		{
			packet[i] = decodeByte(packet[i], 0, key, V2_OFFSET(i, packet[0], V2_OFFSET_JUMP_START));
		}

		if (id == previousID)
		{
			//ESP_LOGI(TAG, "ID %d id==previousID", id);
			return false;
		}
		previousID = id;

		//ESP_LOG_BUFFER_HEXDUMP("DECRYP", packet, 9, ESP_LOG_INFO);
		//Meine Fernbedienung hat ID 040 004
		
		*cmd=packet[4];
		*arg=packet[5];
		return true;
	}
};
