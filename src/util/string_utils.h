#ifndef AMDAHL_SRC_UTIL_STRING_UTILS_H_
#define AMDAHL_SRC_UTIL_STRING_UTILS_H_

const char* const kWhiteSpaceChars = " \t\n\v\f\r";
constexpr char kNullChar = '\0';
constexpr char kColonChar = ':';

#include <algorithm>
#include <string>

inline void GetNextEntryStartEnd(const char* str, int len, int& start, int& end) {
    while (start < len && ('\t' == *(str + start) || ' ' == *(str + start))) {
        start += 1;
    }

    end = start + 1;
    while (end < len && ('\t' != *(str + end) && ' ' != *(str + end))) {
        end += 1;
    }
}

inline bool CaseInsensitiveStrEqual(const std::string& str1, const std::string& str2) {
    if (str1.size() != str2.size()) {
        return false;
    }

    return std::equal(str1.begin(), str1.end(), str2.begin(),
                      [](char a, char b) { return std::tolower(a) == std::tolower(b); });
}

inline std::string ToLower(const std::string& str1) {
    std::string str = str1;
    std::transform(str1.begin(), str1.end(), str.begin(), ::tolower);

    return str;
}

#endif