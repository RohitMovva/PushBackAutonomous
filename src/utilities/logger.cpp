#include "utilities/logger.hpp"
#include <cstdio>
#include <cstdarg>
#include <ctime>
#include "api.h"

// Initialize static member
Logger *Logger::instance = nullptr;

Logger::LogMessage::LogMessage(const std::string &level, const std::string &msg, uint32_t time)
    : logLevel(level), message(msg), timestamp(time) {}

Logger::Logger() : loggerTask(nullptr), isRunning(false)
{
    isSDCardMounted = false;
    generateLogFilename();
    initializeSDCard();
    startLoggingTask();
}

void Logger::generateLogFilename()
{
    uint32_t currentTime = pros::millis();
    time_t rawtime = currentTime / 1000;
    struct tm *timeinfo;
    char buffer[80];
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%Y%m%d_%H%M%S", timeinfo);
    logFilename = "/usd/robot_log_" + std::string(buffer) + ".txt";
}

void Logger::initializeSDCard()
{
    if (pros::usd::is_installed())
    {
        isSDCardMounted = true;
        logFile = fopen(logFilename.c_str(), "a");
        if (logFile != nullptr)
        {
            writeToFile("INFO", "Logger initialized successfully");
            writeToFile("INFO", "Log file: " + logFilename);
        }
        else
        {
            printf("Failed to open log file: %s\n", logFilename.c_str());
        }
    }
    else
    {
        printf("SD card not installed\n");
    }
}

void Logger::startLoggingTask()
{
    isRunning = true;
    loggerTask = new pros::Task([this]
                                { processLogQueue(); });
}

void Logger::processLogQueue()
{
    while (isRunning)
    {
        queueMutex.take();
        if (!messageQueue.empty())
        {
            LogMessage msg = messageQueue.front();
            messageQueue.pop();
            queueMutex.give();

            // Format timestamp
            uint32_t millis = msg.timestamp;
            uint32_t seconds = millis / 1000;
            uint32_t minutes = seconds / 60;
            uint32_t hours = minutes / 60;

            // Write to file with timestamp and log level
            if (logFile != nullptr)
            {
                fprintf(logFile, "[%02d:%02d:%02d.%03d] [%s] %s\n",
                        (int)hours % 24,
                        (int)minutes % 60,
                        (int)seconds % 60,
                        (int)millis % 1000,
                        msg.logLevel.c_str(),
                        msg.message.c_str());
                fflush(logFile);
            }
        }
        else
        {
            queueMutex.give();
            pros::delay(10); // Sleep to prevent CPU hogging
        }
    }
}

void Logger::writeToFile(const std::string &level, const std::string &message)
{
    queueMutex.take();
    messageQueue.emplace(level, message, pros::millis());
    queueMutex.give();
}

Logger *Logger::getInstance()
{
    if (instance == nullptr)
    {
        instance = new Logger();
    }
    return instance;
}

void Logger::log(const char *format, ...)
{
    if (!isSDCardMounted)
        return;
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    writeToFile("LOG", std::string(buffer));
}

void Logger::logError(const char *format, ...)
{
    if (!isSDCardMounted)
        return;
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    writeToFile("ERROR", std::string(buffer));
}

void Logger::logInfo(const char *format, ...)
{
    if (!isSDCardMounted)
        return;
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    writeToFile("INFO", std::string(buffer));
}

void Logger::logDebug(const char *format, ...)
{
    if (!isSDCardMounted)
        return;
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    writeToFile("DEBUG", std::string(buffer));
}

void Logger::logWarning(const char *format, ...)
{
    if (!isSDCardMounted)
        return;
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    writeToFile("WARNING", std::string(buffer));
}

const std::string &Logger::getLogFilename() const
{
    return logFilename;
}

void Logger::close()
{
    if (isRunning)
    {
        isRunning = false;
        if (loggerTask != nullptr)
        {
            loggerTask->remove();
            delete loggerTask;
            loggerTask = nullptr;
        }
    }
    if (logFile != nullptr)
    {
        fclose(logFile);
        logFile = nullptr;
    }
}

Logger::~Logger()
{
    close();
}