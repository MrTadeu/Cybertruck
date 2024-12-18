#pragma once

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
    switch (baud_rate)
    {
    case 1200:
        return LibSerial::BaudRate::BAUD_1200;
    case 1800:
        return LibSerial::BaudRate::BAUD_1800;
    case 2400:
        return LibSerial::BaudRate::BAUD_2400;
    case 4800:
        return LibSerial::BaudRate::BAUD_4800;
    case 9600:
        return LibSerial::BaudRate::BAUD_9600;
    case 19200:
        return LibSerial::BaudRate::BAUD_19200;
    case 38400:
        return LibSerial::BaudRate::BAUD_38400;
    case 57600:
        return LibSerial::BaudRate::BAUD_57600;
    case 115200:
        return LibSerial::BaudRate::BAUD_115200;
    case 230400:
        return LibSerial::BaudRate::BAUD_230400;
    default:
        std::cerr << "Error! Baud rate " << baud_rate << " not supported! Defaulting to 57600" << std::endl;
        return LibSerial::BaudRate::BAUD_57600;
    }
}

class ArduinoComms
{
public:
    int attempt = 0;
    ArduinoComms() : connected_(false) {}

    bool connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        serial_device_ = serial_device;
        baud_rate_ = baud_rate;
        timeout_ms_ = timeout_ms;

        try
        {
            serial_conn_.Open(serial_device_);
            serial_conn_.SetBaudRate(convert_baud_rate(baud_rate_));
            connected_ = true;
            std::cout << "Successfully connected to: " << serial_device_ << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to connect to " << serial_device_ << ": " << e.what() << std::endl;
            connected_ = false;
        }

        return connected_;
    }

    void disconnect()
    {
        if (serial_conn_.IsOpen())
        {
            serial_conn_.Close();
            connected_ = false;
            std::cout << "Disconnected from: " << serial_device_ << std::endl;
        }
        else
        {
            std::cout << "Serial connection already closed." << std::endl;
        }
    }

    bool is_connected() const { return connected_; }

    void reconnect(int max_attempts = 10, int retry_delay_ms = 500)
    {
        while (attempt < max_attempts && !connected_)
        {
            std::cout << "Reconnection attempt " << (attempt + 1)
                      << " of " << max_attempts << " to: " << serial_device_ << std::endl;

            connected_ = connect(serial_device_, baud_rate_, timeout_ms_);

            if (!connected_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
            }
            attempt++;
        }
    }

    void send_msg(const std::string &msg)
    {
        if (!connected_)
        {
            reconnect();
            if (!connected_)
            {
                throw std::runtime_error("Unable to send message: Not connected to hardware.");
            }
        }

        serial_conn_.Write(msg);
    }

    std::string receive_msg()
    {
        if (!connected_)
        {
            throw std::runtime_error("Cannot receive message: Not connected to hardware.");
        }

        std::string response;
        char buffer;
        bool message_complete = false;

        while (!message_complete)
        {
            if (serial_conn_.IsDataAvailable())
            {
                serial_conn_.ReadByte(buffer, timeout_ms_);

                if (buffer >= 32 && buffer <= 126) // Filtra apenas caracteres válidos
                {
                    response += buffer;
                }

                if (buffer == '\n') // Final da mensagem
                {
                    message_complete = true;
                }
            }
        }

        return response;
    }

private:
    LibSerial::SerialPort serial_conn_;
    std::string serial_device_;
    int32_t baud_rate_;
    int32_t timeout_ms_;
    bool connected_;
};
