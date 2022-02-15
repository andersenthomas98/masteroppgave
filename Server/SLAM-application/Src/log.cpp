#include "log.h"
#include <filesystem>
#include <iostream>
#include <string>

#include <iomanip>
#include <ctime>
#include <chrono>


#define LOG_FILE_NAME "_server_logfile"

std::string get_current_date_string() {
	std::time_t const now_c = std::time(0);



	char str[26] = {};
	ctime_s(str, 26, &now_c);

	std::string cppstr = str;

	
	std::string month = cppstr.substr(4, 3);
	std::string day = cppstr.substr(8, 2);
	std::string hour = cppstr.substr(11, 2);
	std::string min = cppstr.substr(14, 2);
	std::string sec = cppstr.substr(17, 2);
	std::string year = cppstr.substr(20, 4);

	return day + "_" + month + "_" + year + "_" + hour + "." + min;

}

std::string generate_log_file_name(){
		int file_number = 1;
		auto path = std::filesystem::current_path().append("\logs");
		for (const auto& entry : std::filesystem::directory_iterator(path)) {
			auto path_string = entry.path().string();

			size_t log_number_string_placement = path_string.find("logs") + 5;			
			std::string filename = path_string.substr(log_number_string_placement);

			std::string number_string = "";
			int i = 0;
			while (filename.substr(i, 1) != "_"){
				number_string.append(filename.substr(i, 1));
				i++;
			}
	
			int number = std::stoi(number_string);
			if (number >= file_number) {
				file_number = number + 1;
			}
		}
		return  +"logs/" + std::to_string(file_number) + LOG_FILE_NAME + "_" + get_current_date_string() + ".txt";
}

namespace NTNU::utility {

	std::shared_ptr<spdlog::logger> log::console_logger;
	std::shared_ptr<spdlog::logger> log::file_logger;

	void log::init()
	{
		spdlog::set_pattern("%^[%T] %l: %v%$");
		console_logger = spdlog::stdout_color_mt("console_logger");
		console_logger->set_level(spdlog::level::trace);


		spdlog::set_pattern("%^[%T] %l: %v%$");


		std::string file_name = generate_log_file_name();  

		file_logger = spdlog::basic_logger_mt("file_logger", file_name);

		LOG_INFO(get_current_date_string());
	}

}

