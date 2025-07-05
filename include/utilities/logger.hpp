/**
 * @file logger.h
 * @brief Thread-safe logging system for robotics applications
 * @author Rohit Movva
 * @date 2025
 */

 #ifndef LOGGER_H
 #define LOGGER_H
 
 #include <string>
 #include <queue>
 #include <cstdio>
 #include "pros/rtos.hpp"
 
 /**
  * @class Logger
  * @brief Singleton thread-safe logger class for writing timestamped messages to SD card
  * 
  * This class provides a robust logging system that writes messages to an SD card
  * with proper timestamps and log levels. It uses a separate task for file I/O
  * operations to prevent blocking the main thread.
  * 
  * @note This class follows the Singleton pattern - use getInstance() to get the instance
  * 
  * Example usage:
  * @code
  * Logger* logger = Logger::getInstance();
  * logger->logInfo("System initialized");
  * logger->logError("Sensor connection failed: %s", errorMsg);
  * @endcode
  */
 class Logger {
 private:
     /**
      * @struct LogMessage
      * @brief Internal structure for storing log messages in the queue
      */
     struct LogMessage {
         std::string logLevel;    ///< Log level (INFO, DEBUG, WARNING, ERROR, LOG)
         std::string message;     ///< The actual log message
         uint32_t timestamp;      ///< Timestamp in milliseconds
         
         /**
          * @brief Constructor for LogMessage
          * @param level The log level string
          * @param msg The message content
          * @param time Timestamp in milliseconds
          */
         LogMessage(const std::string& level, const std::string& msg, uint32_t time);
     };
 
     static Logger* instance;           ///< Singleton instance pointer
     std::queue<LogMessage> messageQueue; ///< Queue for storing log messages
     pros::Mutex queueMutex;           ///< Mutex for thread-safe queue access
     pros::Task* loggerTask;           ///< Background task for file I/O
     std::string logFilename;          ///< Current log file path
     FILE* logFile;                    ///< File handle for log file
     bool isSDCardMounted;             ///< Flag indicating if SD card is available
     bool isRunning;                   ///< Flag for controlling the logging task
 
     /**
      * @brief Private constructor (Singleton pattern)
      * 
      * Initializes the logger, creates log filename, mounts SD card,
      * and starts the background logging task.
      */
     Logger();
 
     /**
      * @brief Generates a unique filename based on current timestamp
      * 
      * Creates a filename in the format: /usd/robot_log_YYYYMMDD_HHMMSS.txt
      */
     void generateLogFilename();
 
     /**
      * @brief Initializes SD card and opens log file
      * 
      * Checks if SD card is installed, opens the log file for writing,
      * and writes initialization messages.
      */
     void initializeSDCard();
 
     /**
      * @brief Starts the background logging task
      * 
      * Creates a new PROS task that runs processLogQueue() continuously.
      */
     void startLoggingTask();
 
     /**
      * @brief Main loop for the logging task
      * 
      * Continuously processes messages from the queue and writes them to file.
      * Runs in a separate task to prevent blocking the main thread.
      */
     void processLogQueue();
 
     /**
      * @brief Adds a message to the logging queue
      * @param level Log level string (INFO, DEBUG, WARNING, ERROR, LOG)
      * @param message The message to log
      * 
      * Thread-safe method that adds messages to the queue for processing
      * by the background task.
      */
     void writeToFile(const std::string& level, const std::string& message);
 
 public:
     /**
      * @brief Gets the singleton instance of Logger
      * @return Pointer to the Logger instance
      * 
      * Creates the instance if it doesn't exist. This is the only way
      * to obtain a Logger instance.
      */
     static Logger* getInstance();
 
     /**
      * @brief Logs a general message
      * @param format Printf-style format string
      * @param ... Variable arguments for the format string
      * 
      * Logs a message with "LOG" level. Uses printf-style formatting.
      * 
      * Example:
      * @code
      * logger->log("Motor speed: %d RPM", motorSpeed);
      * @endcode
      */
     void log(const char* format, ...);
 
     /**
      * @brief Logs an error message
      * @param format Printf-style format string
      * @param ... Variable arguments for the format string
      * 
      * Logs a message with "ERROR" level. Use for critical errors
      * that need immediate attention.
      * 
      * Example:
      * @code
      * logger->logError("Sensor %d failed with code %d", sensorId, errorCode);
      * @endcode
      */
     void logError(const char* format, ...);
 
     /**
      * @brief Logs an informational message
      * @param format Printf-style format string
      * @param ... Variable arguments for the format string
      * 
      * Logs a message with "INFO" level. Use for general information
      * about system state and operations.
      * 
      * Example:
      * @code
      * logger->logInfo("System initialized successfully");
      * @endcode
      */
     void logInfo(const char* format, ...);
 
     /**
      * @brief Logs a debug message
      * @param format Printf-style format string
      * @param ... Variable arguments for the format string
      * 
      * Logs a message with "DEBUG" level. Use for detailed debugging
      * information during development.
      * 
      * Example:
      * @code
      * logger->logDebug("Variable state: x=%d, y=%f", x, y);
      * @endcode
      */
     void logDebug(const char* format, ...);
 
     /**
      * @brief Logs a warning message
      * @param format Printf-style format string
      * @param ... Variable arguments for the format string
      * 
      * Logs a message with "WARNING" level. Use for non-critical issues
      * that should be noted but don't prevent operation.
      * 
      * Example:
      * @code
      * logger->logWarning("Memory usage at %d%%, consider cleanup", memUsage);
      * @endcode
      */
     void logWarning(const char* format, ...);
 
     /**
      * @brief Gets the current log filename
      * @return Reference to the log filename string
      * 
      * Returns the full path to the current log file being written to.
      */
     const std::string& getLogFilename() const;
 
     /**
      * @brief Closes the logger and cleans up resources
      * 
      * Stops the logging task, closes the file handle, and cleans up
      * allocated resources. Called automatically by destructor.
      */
     void close();
 
     /**
      * @brief Destructor
      * 
      * Ensures proper cleanup by calling close().
      */
     ~Logger();
 
     // Prevent copying (Singleton pattern)
     Logger(const Logger&) = delete;            ///< Copy constructor deleted
     Logger& operator=(const Logger&) = delete; ///< Assignment operator deleted
 };
 
 #endif // LOGGER_H