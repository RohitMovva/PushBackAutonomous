#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <queue>
#include "pros/rtos.hpp"

class Logger {
private:
    static Logger* instance;
    FILE* logFile;
    bool isSDCardMounted;
    std::string logFilename;
    
    // Thread management
    pros::Task* loggerTask;
    bool isRunning;
    
    // Message queue
    struct LogMessage {
        bool isError;
        std::string message;
        uint32_t timestamp;
        
        LogMessage(bool error, const std::string& msg, uint32_t time);
    };
    
    std::queue<LogMessage> messageQueue;
    pros::Mutex queueMutex;
    
    // Private constructor for singleton pattern
    Logger();
    
    // Prevent copying and assignment
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    
    void generateLogFilename();
    void initializeSDCard();
    void startLoggingTask();
    void processLogQueue();
    void writeToFile(bool isError, const std::string& message);

public:
    static Logger* getInstance();
    void log(const char* format, ...);
    void logError(const char* format, ...);
    const std::string& getLogFilename() const;
    void close();
    ~Logger();
};

#endif // LOGGER_H