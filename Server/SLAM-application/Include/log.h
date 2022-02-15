#pragma once

#pragma once

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h" // support for basic file logging
#include "spdlog/sinks/stdout_color_sinks.h"

#include <memory>


/*
	Log levels:
	0 - console log nothing
	1 - console log only critical
	2 - console log error and above
	3 - console log warn and above
	4 - console log info and above
	5 - console log all, including trace 
	
	Everything will be logged to files. 
*/
#define LOG_LEVEL 4 

namespace NTNU::utility {


	class log
	{
	public:
		static void init();

		inline static std::shared_ptr<spdlog::logger>& get_console_logger() { return console_logger; }
		inline static std::shared_ptr<spdlog::logger>& get_file_logger() { return file_logger; }

	private:
		static std::shared_ptr<spdlog::logger> console_logger;
		static std::shared_ptr<spdlog::logger> file_logger;
	};
}


// File logging macros
#define FILE_LOGGER         ::NTNU::utility::log::get_file_logger()
#define LOG_FILE(...)       FILE_LOGGER->info(__VA_ARGS__);

// Console logging macros, log to console and file. 
#define LOG_TRACE(...)      ::NTNU::utility::log::get_console_logger()->trace(__VA_ARGS__);    FILE_LOGGER->trace(__VA_ARGS__)
#define LOG_INFO(...)       ::NTNU::utility::log::get_console_logger()->info(__VA_ARGS__);     FILE_LOGGER->info(__VA_ARGS__)
#define LOG_WARN(...)       ::NTNU::utility::log::get_console_logger()->warn(__VA_ARGS__);     FILE_LOGGER->warn(__VA_ARGS__)
#define LOG_ERROR(...)      ::NTNU::utility::log::get_console_logger()->error(__VA_ARGS__);    FILE_LOGGER->error(__VA_ARGS__)
#define LOG_CRITICAL(...)   ::NTNU::utility::log::get_console_logger()->critical(__VA_ARGS__); FILE_LOGGER->critical(__VA_ARGS__)

// Only file logging for console loggers disabled by LOG_LEVEL
#if LOG_LEVEL < 5
	#define LOG_TRACE(...)    FILE_LOGGER->trace(__VA_ARGS__)
#endif 
#if LOG_LEVEL < 4
	#define LOG_INFO(...)     FILE_LOGGER->info(__VA_ARGS__)
#endif
#if LOG_LEVEL < 3
	#define LOG_WARN(...)     FILE_LOGGER->warn(__VA_ARGS__)
#endif
#if LOG_LEVEL < 2
	#define LOG_ERROR(...)    FILE_LOGGER->error(__VA_ARGS__)
#endif
#if LOG_LEVEL < 1
	#define LOG_CRITICAL(...) FILE_LOGGER->critical(__VA_ARGS__)
#endif


