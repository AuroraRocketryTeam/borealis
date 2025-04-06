#include "E220LoRaTransmitter.hpp"
#include "global/pins.h"

/**
 * @brief Initializa the LoRa module with default configuration.
 * 
 * @return ResponseStatusContainer 
 */
ResponseStatusContainer E220LoRaTransmitter::init()
{
    transmitter.begin();
    auto configurationStatus = this->getConfiguration();
    auto configuration = *(Configuration *)configurationStatus.data;

    configuration.ADDL = 0x03;
    configuration.ADDH = 0x00;

    configuration.CHAN = 23;

    configuration.SPEED.uartBaudRate = getBpsType();
    configuration.SPEED.airDataRate = AIR_DATA_RATE_111_625;
    configuration.SPEED.uartParity = MODE_00_8N1;

    configuration.OPTION.subPacketSetting = SPS_200_00;
    configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
    configuration.OPTION.transmissionPower = POWER_22;

    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
    configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
    configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

    configurationStatus.close();
    return this->configure(configuration);
}

ResponseStatusContainer E220LoRaTransmitter::init(Configuration config)
{
    transmitter.begin();
    return this->configure(config);
}

/**
 * @brief Converts the data to a byte array.
 *
 * @param data
 * @return std::vector<uint8_t>
 */
std::vector<uint8_t> convertToByteArray(const TransmitDataType &data)
{
    std::string sData;

    if (std::holds_alternative<char *>(data))
    {
        sData = std::get<char *>(data);
    }
    else if (std::holds_alternative<String>(data))
    {
        sData = std::get<String>(data).c_str();
    }
    else if (std::holds_alternative<std::string>(data))
    {
        sData = std::get<std::string>(data);
    }
    else if (std::holds_alternative<nlohmann::json>(data))
    {
        sData = std::get<nlohmann::json>(data).dump();
    }
    else
    {
        return {};
    }

    return std::vector<uint8_t>(sData.begin(), sData.end());
}

std::vector<uint8_t> compressData(std::vector<uint8_t> data)
{
    // use zstd to compress the data
    size_t compressedSize = ZSTD_compressBound(data.size());
    std::vector<uint8_t> compressedData(compressedSize);

    compressedSize = ZSTD_compress(compressedData.data(), compressedSize, data.data(), data.size(), 1);

    if (ZSTD_isError(compressedSize))
    {
        return {};
    }

    compressedData.resize(compressedSize);
    return compressedData;
}

ResponseStatusContainer E220LoRaTransmitter::transmit(TransmitDataType data)
{
    std::vector<uint8_t> byteArray = convertToByteArray(data);
    if (byteArray.empty())
    {
        return ResponseStatusContainer(ERR_E220_UNKNOWN, "Couldn't convert data to byte array. Wrong data type or empty buffer.");
    }

    std::vector<uint8_t> compressedData = compressData(byteArray);
    size_t dataLength = compressedData.size();
    size_t offset = 0;

    // Preparazione del pacchetto
    Packet packet = {};
    packet.header.packetNumber = this->packetNumber++;
    // Calcolo del numero totale di chunk per rientrare nei 199 byte massimi.
    packet.header.totalChunks = (dataLength + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;
    packet.header.chunkNumber = 1;

    while (offset < dataLength)
    {
        packet.header.timestamp = static_cast<uint32_t>(time(nullptr));
        uint8_t payloadSize = std::min(dataLength - offset, static_cast<size_t>(MAX_PAYLOAD_SIZE));
        // Dimensione del payload
        packet.header.payloadSize = payloadSize;
        // Dimensione totale del pacchetto (TODO: Valutare di padding per arrivare sempre a 199 byte)
        packet.header.chunkSize = payloadSize + sizeof(PacketHeader) + sizeof(uint16_t);

        // Suddivisione del pacchetto in chunk
        memcpy(packet.payload.data, compressedData.data() + offset, payloadSize);

        packet.calculateCRC();

        auto rc = this->transmitter.sendFixedMessage(LORA_RECEIVER_ADDH, LORA_RECEIVER_ADDL, LORA_CHANNEL,
                                                     &packet, sizeof(Packet));
        // Delay per evitare collisioni, empiricamente il minimo è 30ms.
        delay(30);

        if (rc.code != E220_SUCCESS)
        {
            return ResponseStatusContainer(rc.code, rc.getResponseDescription());
        }
        packet.header.chunkNumber++;
        offset += payloadSize;       // Sposta l'offset al prossimo chunk
    }

    return ResponseStatusContainer(E220_SUCCESS, "Data sent successfully.");
}

ResponseStatusContainer E220LoRaTransmitter::configure(Configuration configuration)
{
    auto response = transmitter.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    return ResponseStatusContainer(response.code, response.getResponseDescription());
}

ResponseStructContainer E220LoRaTransmitter::getConfiguration()
{
    return transmitter.getConfiguration();
}

String E220LoRaTransmitter::getConfigurationString(Configuration configuration) const
{
    String result;
    result += "----------------------------------------\n";

    result += "HEAD : " + String(configuration.COMMAND, HEX) + " " + String(configuration.STARTING_ADDRESS, HEX) + " " + String(configuration.LENGHT, HEX) + "\n\n";
    result += "AddH : " + String(configuration.ADDH, HEX) + "\n";
    result += "AddL : " + String(configuration.ADDL, HEX) + "\n\n";
    result += "Chan : " + String(configuration.CHAN, DEC) + " -> " + configuration.getChannelDescription() + "\n\n";
    result += "SpeedParityBit     : " + String(configuration.SPEED.uartParity, BIN) + " -> " + configuration.SPEED.getUARTParityDescription() + "\n";
    result += "SpeedUARTDatte     : " + String(configuration.SPEED.uartBaudRate, BIN) + " -> " + configuration.SPEED.getUARTBaudRateDescription() + "\n";
    result += "SpeedAirDataRate   : " + String(configuration.SPEED.airDataRate, BIN) + " -> " + configuration.SPEED.getAirDataRateDescription() + "\n\n";
    result += "OptionSubPacketSett: " + String(configuration.OPTION.subPacketSetting, BIN) + " -> " + configuration.OPTION.getSubPacketSetting() + "\n";
    result += "OptionTranPower    : " + String(configuration.OPTION.transmissionPower, BIN) + " -> " + configuration.OPTION.getTransmissionPowerDescription() + "\n";
    result += "OptionRSSIAmbientNo: " + String(configuration.OPTION.RSSIAmbientNoise, BIN) + " -> " + configuration.OPTION.getRSSIAmbientNoiseEnable() + "\n\n";
    result += "TransModeWORPeriod : " + String(configuration.TRANSMISSION_MODE.WORPeriod, BIN) + " -> " + configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription() + "\n";
    result += "TransModeEnableLBT : " + String(configuration.TRANSMISSION_MODE.enableLBT, BIN) + " -> " + configuration.TRANSMISSION_MODE.getLBTEnableByteDescription() + "\n";
    result += "TransModeEnableRSSI: " + String(configuration.TRANSMISSION_MODE.enableRSSI, BIN) + " -> " + configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription() + "\n";
    result += "TransModeFixedTrans: " + String(configuration.TRANSMISSION_MODE.fixedTransmission, BIN) + " -> " + configuration.TRANSMISSION_MODE.getFixedTransmissionDescription() + "\n";

    result += "----------------------------------------\n";

    return result;
}

template <typename T>
T E220LoRaTransmitter::getBpsValue(const std::map<int, T> &baudRateMap, T defaultValue) const
{
    auto it = baudRateMap.find(SERIAL_BAUD_RATE);
    if (it != baudRateMap.end())
    {
        return it->second;
    }
    else
    {
        return defaultValue;
    }
}

UART_BPS_RATE E220LoRaTransmitter::getBpsRate() const
{
    static const std::map<int, UART_BPS_RATE> baudRateMap = {
        {1200, UART_BPS_RATE_1200},
        {2400, UART_BPS_RATE_2400},
        {4800, UART_BPS_RATE_4800},
        {9600, UART_BPS_RATE_9600},
        {19200, UART_BPS_RATE_19200},
        {38400, UART_BPS_RATE_38400},
        {57600, UART_BPS_RATE_57600},
        {115200, UART_BPS_RATE_115200}};

    return getBpsValue(baudRateMap, UART_BPS_RATE_9600);
}

UART_BPS_TYPE E220LoRaTransmitter::getBpsType() const
{
    static const std::map<int, UART_BPS_TYPE> baudRateMap = {
        {1200, UART_BPS_1200},
        {2400, UART_BPS_2400},
        {4800, UART_BPS_4800},
        {9600, UART_BPS_9600},
        {19200, UART_BPS_19200},
        {38400, UART_BPS_38400},
        {57600, UART_BPS_57600},
        {115200, UART_BPS_115200}};

    return getBpsValue(baudRateMap, UART_BPS_9600);
}