#ifndef AMDAHL_SRC_UTIL_DATE_UTILS_H_
#define AMDAHL_SRC_UTIL_DATE_UTILS_H_

#include <iomanip>
#include <sstream>
#include <string>

const char* const kDateFormat = "%Y-%m-%d";

static inline std::string DateToStr(const std::tm& date) {
    std::stringstream date_str;
    date_str << std::put_time(&date, kDateFormat);

    return date_str.str();
}

static inline std::string TimeToDateStr(std::time_t time) {
    std::tm tm = *std::localtime(&time);
    return DateToStr(tm);
}

static inline std::string GetCurrentDateStr() {
    std::time_t now = std::time(nullptr);
    return TimeToDateStr(now);
}

#endif