#include <Arduino.h>
#include <Constants.h>
#include <Panel.h>

Panel::Panel(long baudRate) : baudRate(baudRate) {
    // Constructor implementation (if needed)
}
void Panel::begin() {
    Serial.begin(baudRate);
}

int Panel::waitReady(unsigned long timeoutDuration) {
    unsigned long startTime = millis();

    while (millis() - startTime <= timeoutDuration) {
        if (messageAvailable()) { // Check if a message is available
            String receivedMessage = readMessage(); // Use the readMessage method
            if (receivedMessage.indexOf(PANEL_READY_MESSAGE) != -1) {
                return 1; // Expected string received
            }
        }
    }
    return 0; // Timeout
}

void Panel::sendMessage(const String &message) {
    Serial.println(message);
}

bool Panel::messageAvailable() {
    return Serial.available() > 0;
}

String Panel::readMessage() {
    if (!messageAvailable()) {
        return ""; // No message available
    }
    String message = Serial.readStringUntil('\n');
    return message;
}

int Panel::handleCommand() {
    // Handle the message and return an appropriate response
    if (!messageAvailable()) {
        return -1; // No message available to handle
    }
    String message = readMessage(); // Read the message
    // Process the message here (e.g., parse commands, etc.)
    switch (message.charAt(0)) // Example: switch based on the first character of the message
    {
    case 'E': // 'E' for End
        sendMessage("End command received");
        return 0; // End command received
        break;

    case 'P': // P for pause
        sendMessage("Pause command received");
        return 1;
        break;

    default: // Unknown command
        sendMessage("Unknown command received: " + String(message.charAt(0)));
        return -1; // Unknown command
        break;
    }
    Serial.println("Received message: " + message);
    return -2; // Placeholder for actual handling logic
}