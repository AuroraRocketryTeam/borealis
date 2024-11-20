#include "E220LoRaTransmitter.hpp"

bool E220LoRaTransmitter::init()
{
    transmitter.begin();
    auto configurationStatus = this->getConfiguration();
    auto configuration = *(Configuration *)configurationStatus.data;

    configuration.ADDL = 0x03;
    configuration.ADDH = 0x00;

    configuration.CHAN = 23;

    configuration.SPED.uartBaudRate = UART_BPS_9600;
    configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
    configuration.SPED.uartParity = MODE_00_8N1;

    configuration.OPTION.subPacketSetting = SPS_200_00;
    configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
    configuration.OPTION.transmissionPower = POWER_22;

    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
    configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
    configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

    configurationStatus.close();
    return this->init(9600, configuration);
}

bool E220LoRaTransmitter::init(unsigned long serialBaudRate, Configuration config)
{
    transmitter.begin();
    auto configurationStatus = this->getConfiguration();
    auto currentConfig = *(Configuration *)configurationStatus.data;
    if (configurationStatus.status.code != E220_SUCCESS)
    {
        logger->logError(("Failed to get configuration from LoRa module with error: " + String(configurationStatus.status.getResponseDescription()) + " (" + String(configurationStatus.status.code) + ")").c_str());
        return false;
    }
    logger->logInfo(("LoRa module status: " + String(configurationStatus.status.getResponseDescription()) + " (" + String(configurationStatus.status.code) + ")").c_str());
    logger->logInfo(("Current configuration: " + getConfigurationString(currentConfig)).c_str());
    configurationStatus.close();
    return this->configure(config);
}

void E220LoRaTransmitter::transmit(std::variant<char *, String, std::string, nlohmann::json> data)
{
    // Extract the data from the variant
    String dataString;
    dataString = std::holds_alternative<char *>(data) ? String(std::get<char *>(data)) :
                 std::holds_alternative<String>(data) ? std::get<String>(data) :
                 std::holds_alternative<std::string>(data) ? String(std::get<std::string>(data).c_str()) :
                 std::holds_alternative<nlohmann::json>(data) ? String(std::get<nlohmann::json>(data).dump().c_str()) :
                 "";
    auto response = transmitter.sendFixedMessage(LORA_DESTINATION_ADDH, LORA_DESTINATION_ADDL, LORA_CHANNEL, dataString);

    if (response.code != E220_SUCCESS)
    {
        logger->logError(("Failed to send message to LoRa module with error: " + String(response.getResponseDescription()) + " (" + String(response.code) + ")").c_str());
        return;
    }
    logger->logInfo("Message sent successfully.");
    logger->logInfo(("Message: " + dataString).c_str());
}

bool E220LoRaTransmitter::configure(Configuration configuration)
{
    auto response = transmitter.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    if (response.code != E220_SUCCESS)
    {
        logger->logError(("Failed to set configuration to LoRa module with error: " + String(response.getResponseDescription()) + " (" + String(response.code) + ")").c_str());
        return false;
    }

    logger->logInfo("Configuration set successfully.");
    logger->logInfo(("New configuration: " + getConfigurationString(*(Configuration *)this->getConfiguration().data)).c_str());
    return true;
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
    result += "SpeedParityBit     : " + String(configuration.SPED.uartParity, BIN) + " -> " + configuration.SPED.getUARTParityDescription() + "\n";
    result += "SpeedUARTDatte     : " + String(configuration.SPED.uartBaudRate, BIN) + " -> " + configuration.SPED.getUARTBaudRateDescription() + "\n";
    result += "SpeedAirDataRate   : " + String(configuration.SPED.airDataRate, BIN) + " -> " + configuration.SPED.getAirDataRateDescription() + "\n\n";
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