#include "Netstrings.hpp"

using namespace std;

string encodedNetString(string plainInput){
	//if the input string is empty, return "error"
	if (plainInput.empty()) throw "String Empty";

	//return a NetString in the form of [len]":"[string]","
	return  to_string(plainInput.length()) + ":" + plainInput + ",";
}

string decodedNetString(string netString){

	//if the NetString is less than 3 characters, it's either an invalid one or contains an empty string
	if (netString.length() < 3) throw "NetString cannot be shorter than 3 characters!";
	int semicolonIndex = netString.find(':');

	// if there's no semicolon, then it's an invalid NetString
	if (semicolonIndex < 0) throw "NetString contained no semicolon!";

	//parse until the semicolon. Those should be the control digits
	string parsedDigits = netString.substr(0, semicolonIndex);
	int controlDigits = std::stoi(parsedDigits);

	//if the control digit is smaller than 1, then it's either not a digit or the NetString is invalid
	if (controlDigits < 1) throw "ControlDigit is smaller than 1";

	//parse after the semicolon until the end of the string
	string command = netString.substr(semicolonIndex+1);

	// if it's an empty string, return "error"
	if (command.empty()) throw "NetString empty";

	//if last character is a comma, remove it
	if (command.substr(command.length() -1) == ",") command.erase(command.length()-1);

	//if string's length isn't equal with the control digits, it's an invalid NetString
    int commandLength = command.length();
	if (commandLength != controlDigits) throw "String does not correspond to controlDigit";

	return command;
}
