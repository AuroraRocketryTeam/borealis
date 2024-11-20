#pragma once

#include <nlohmann/json.hpp>
#include <variant>
#include <string>
#include <Arduino.h>

/**
 * @brief Interface for a transmitter
 * 
 */
class ITransmitter
{
    public:
        virtual bool init() = 0;
        virtual void transmit(std::variant<char *, String, std::string, nlohmann::json> data) = 0;
};

