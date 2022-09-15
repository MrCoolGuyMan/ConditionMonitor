#ifndef PDO_INSTRUCTION_SET_h
#define PDO_INSTRUCTION_SET_h

enum PDO_COMMAND_SET{
    PDO_RAM_TRANSFER = 0,
    PDO_CLEAR_MEMORY = 1,
    PDO_TRANSFER_SENSOR_READING = 2,
    PDO_ENTER_SENSOR_MONITORING_MODE = 3,
    PDO_TOGGLE_MOTOR = 4,
    
    // This must be last!
    PDO_TOTAL_INSTRUCTIONS
};

enum PDO_STATUS{
    PDO_COMMAND_COMPLETE = 1,
    PDO_ERROR_GENERIC
};
#endif