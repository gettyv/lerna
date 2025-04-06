#ifndef PANEL_H
#define PANEL_H

#include <Arduino.h>

class Panel {
public:
    // Constructor
    Panel(long baudRate);

    // Initialize serial communication
    void begin();

    int waitReady(unsigned long timeoutDuration);


    void sendMessage(const String &message);

    int handleCommand();


    bool messageAvailable();

    String readMessage();

private:
    long baudRate;
};

#endif // PANEL_H
