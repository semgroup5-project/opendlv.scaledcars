#pragma once
#include <string>

/* encodedNetString accepts a String as an argument and returns the equivalent NetString. If an empty string is used, then the string "error" is returned */
std::string encodedNetString(std::string plainInput);

/* decodedNetString accepts a NetString as an argument (WITHOUT the trailing ",") and returns the equivalent string. If an invalid NetString is used, then the string "error" is returned */
std::string decodedNetString(std::string netString);
