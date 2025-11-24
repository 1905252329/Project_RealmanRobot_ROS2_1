#include <modbus/modbus.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cerrno>


class GripperController {
    public:
        GripperController(const std::string& serial_port, int baud_rate = 115200, char parity = 'N', int data_bits = 8, int stop_bits = 1)
            : mb(nullptr), port(serial_port), baud(baud_rate), parity(parity), data_bits(data_bits), stop_bits(stop_bits) {}
    
        ~GripperController() {
            if (mb) {
                modbus_close(mb);
                modbus_free(mb);
            }
        }
    
        bool connect(int slave_id) {
            mb = modbus_new_rtu(port.c_str(), baud, parity, data_bits, stop_bits);
            if (!mb) {
                std::cerr << "Failed to create Modbus context for port: " << port << "\n";
                return false;
            }
    
            modbus_set_slave(mb, slave_id);
    
            if (modbus_connect(mb) == -1) {
                std::cerr << "Failed to connect to Modbus slave on port: " << port << "\n";
                modbus_free(mb);
                mb = nullptr;
                return false;
            }
    
            return true;
        }
    
        bool activateGripper(uint16_t activation_register) {
            uint16_t command = 0x0001; // Activation command
            if (modbus_write_register(mb, activation_register, command) == -1) {
                std::cerr << "Failed to activate gripper on port: " << port << "\n";
                return false;
            }
            return true;
        }
    
        bool setGripperPosition(uint16_t position_value) {
            const uint16_t position_register = 0x0103;
            const uint16_t status_register = 0x0201;
            const int max_attempts = 50;
            const int delay_ms = 100;
        
            // if (modbus_write_register(mb, position_register, position_value) == -1) {
            //     std::cerr << "Failed to write position to gripper on port: " << port << "\n";
            //     return false;
            // }
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                // Write target position
                if (modbus_write_register(mb, position_register, position_value) == -1) {
                    std::cerr << "Failed to write position to gripper on port: " << port 
                    << ", reason: " << modbus_strerror(errno) << "\n";
                    continue;
                    // return false;
                }
        
                // Read gripper state
                uint16_t status = 0;
                if (modbus_read_registers(mb, status_register, 1, &status) == -1) {
                    std::cerr << "Failed to read status register on port: " << port 
                    << ", reason: " << modbus_strerror(errno) << "\n";
                    continue;
                    // return false;
                }
        
                // Check if position is reached
                if (status == 1 || status == 2) {
                    std::cout << "Gripper action complete (status: " << status << ").\n";
                    return true;
                }
        
                // Wait before next attempt
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            }
        
            std::cerr << "Gripper did not reach the target position within the expected time.\n";
            return false;
        }
        
    
    private:
        modbus_t* mb;
        std::string port;
        int baud;
        char parity;
        int data_bits;
        int stop_bits;
    };