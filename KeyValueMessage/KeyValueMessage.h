#ifndef KeyValueMessage_h
#define KeyValueMessage_h

#include "Arduino.h"
#include <StandardCplusplus.h>
#include <vector>
#include <string>


struct KeyValuePair
{
   String key;
   String value;
};

class KeyValueMessage
{
  public:
    KeyValueMessage(char separator = ' ') { _separator = separator; }
    String KeyValueString(String key, String value);
    String KeyValueAsIntString(String key, int value);
    String KeyValueAsDoubleString(String key, double value);
    String KeyValueAsBoolString(String key, bool value);
    String ValueOf(String key, String message);
    std::vector<KeyValuePair> AllKeyValues(String message);
    std::vector<KeyValuePair> AllKeyValues(std::string message);
    static bool ConvertToBool(String value);
    bool KeyIsPresent(String key, String& message);
    bool ValueAsInt(String key, String& message, int* dstIntValuePtr);
    bool ValueAsDouble(String key, String& message, double* dstDoubleValuePtr);

  private:
    char _separator;
};

#endif
