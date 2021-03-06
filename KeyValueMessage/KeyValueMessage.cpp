#include "Arduino.h"
#include "KeyValueMessage.h"

using namespace std;

String KeyValueMessage::KeyValueString(String key, String value)
{
  String keyValueStr = key;
  keyValueStr += "=";
  keyValueStr += value;
  return keyValueStr;
}

String KeyValueMessage::KeyValueAsIntString(String key, int value)
{
  String keyValueStr = key;
  keyValueStr += "=";
  keyValueStr += value;
  return keyValueStr;
}

String KeyValueMessage::KeyValueAsDoubleString(String key, double value)
{
  String keyValueStr = key;
  keyValueStr += "=";
  char valueArr[15];
  dtostrf(value, 15, 15, valueArr);
  String valueStr = valueArr;
  keyValueStr += valueStr;
  return keyValueStr;
}

String KeyValueMessage::KeyValueAsBoolString(String key, bool value)
{
  String keyValueStr = key;
  keyValueStr += "=";
  keyValueStr += (value == true ? "true" : "false");
  return keyValueStr;
}

String KeyValueMessage::ValueOf(String key, String message)
{
  int keyStart = message.indexOf(key);//StartPositionOfKey(message, key);
  if (keyStart == -1)
	return "";
  if (keyStart == 0 || message.charAt(keyStart - 1) == _separator)
  {
    int separatorPosition = message.indexOf(_separator, keyStart + key.length() + 1);
    if (separatorPosition > -1)
    {
      return message.substring(keyStart + key.length() + 1, separatorPosition);
    }
    else // To the end of the string
      return message.substring(keyStart + key.length() + 1);
    
  }
  else return ValueOf(key, message.substring(keyStart + 1)); // Ex.: "coronation=1 nation=2", key = "nation" -> Truncate the message beginning and search again until the condition is met
}

/*
String KeyValueMessage::AvailableMessage(SoftwareSerial& channel, char endOfMessageChar, unsigned long	   maximumDelayInMilliseconds)
{
  if (channel.available())
  {
    String inputMessage;
    char readChar = channel.read();
    bool endOfMsgCharWasReceived = (readChar == endOfMessageChar);
    bool aNewCharacterIsRead = true;
    unsigned long startTime = millis();
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;
    while(!endOfMsgCharWasReceived && elapsedTime <= maximumDelayInMilliseconds)
    {
      if (aNewCharacterIsRead)
        inputMessage += readChar;
      aNewCharacterIsRead = false;
      if (channel.available())
      {
        readChar = channel.read();
        endOfMsgCharWasReceived = (readChar == endOfMessageChar);
        aNewCharacterIsRead = true;
      }
    currentTime = millis();
    elapsedTime = currentTime - startTime;
    }
    // Remove 'ÿ' at the beginning of the message (as received from the RPi's wiringPi library)
    while (inputMessage.length() > 1 && (int) inputMessage[0] == -1) // The int value of the 1st character is -1
    {
	inputMessage = inputMessage.substring(1); // Remove the 1st character
    }
    return inputMessage;
  }
  else
	return "";
}
*/

std::vector<KeyValuePair> KeyValueMessage::AllKeyValues(String message)
{
   vector<KeyValuePair> keyValuePairsVct;
   // Find the separators: key start are at space (i.e. separator) index + 1
   // "key1=value1 key2=value2 key3=value3 ... "
   //  ^           ^           ^
   vector<int> keyStartPositionsVct;
   keyStartPositionsVct.push_back(0);
   int lastFoundNdx = 0;
   while (lastFoundNdx != -1)
   {
	lastFoundNdx = message.indexOf(_separator, lastFoundNdx + 1);
	if (lastFoundNdx != -1)
	   keyStartPositionsVct.push_back(lastFoundNdx + 1);
   }
   // Find the equal signs and build the key-value pairs
   // "key1=value1 key2=value2 key3=value3 ... "
   //      ^           ^           ^
   
   for (int keyNdx = 0; keyNdx < keyStartPositionsVct.size(); keyNdx++)
   {
	int equalSignPosition = message.indexOf('=', keyStartPositionsVct[keyNdx]);
	if (equalSignPosition != -1);
	{
		KeyValuePair pair;
		pair.key = message.substring(keyStartPositionsVct[keyNdx], equalSignPosition);
		if (keyNdx < keyStartPositionsVct.size() - 1) // Not the last value
			pair.value = message.substring(equalSignPosition + 1, keyStartPositionsVct[keyNdx + 1] - 1);
		else // Last value
			pair.value = message.substring(equalSignPosition + 1);
		keyValuePairsVct.push_back(pair);
	}
   }
   return keyValuePairsVct;
}

std::vector<KeyValuePair> KeyValueMessage::AllKeyValues(std::string message)
{
   String messageAsString(message.c_str());
   return AllKeyValues(messageAsString);
}

bool KeyValueMessage::ConvertToBool(String value)
{
	if (value.equalsIgnoreCase("true"))
		return true;
	else
		return false;
}

bool KeyValueMessage::KeyIsPresent(String key, String& message)
{
	String value = ValueOf(key, message);
	if (value.length() == 0)
		return false;
	else
		return true;
}

bool KeyValueMessage::ValueAsInt(String key, String& message, int* dstIntValuePtr)
{
	if (KeyIsPresent(key, message))
	{
		String value = ValueOf(key, message);
		*dstIntValuePtr = value.toInt();
		return true;
	}
	else
	{
		*dstIntValuePtr = 0;
		return false;
	}
}

bool KeyValueMessage::ValueAsDouble(String key, String& message, double* dstDoubleValuePtr)
{
	if (KeyIsPresent(key, message))
	{
		String value = ValueOf(key, message);
		char buf[value.length()];
		value.toCharArray(buf, value.length());
		double valueDbl = atof(buf); 
		*dstDoubleValuePtr = valueDbl;
		return true;
	}
	else
	{
		*dstDoubleValuePtr = 0;
		return false;
	}
}

