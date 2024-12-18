#include "SoftSerialParser.h"

SoftSerialParser softParser(A0, A1);

void commandHandler(char *tokens[], int tokenCount) {
    Serial.println("SoftSerial Command:");
    for (int i = 0; i < tokenCount; i++) {
        Serial.print("Param ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(tokens[i]);
    }
}

void setup() {
    Serial.begin(9600);
    softParser.begin();
    softParser.setCommandCallback(commandHandler);

    Serial.println("SoftSerialParser Test");
}

void loop() {
    softParser.processSerial();
}
///////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


// #include "HardSerialParser.h"

// HardSerialParser hardParser(Serial);

// void commandHandler(char *tokens[], int tokenCount) {
//     Serial.println("HardSerial Command:");
//     for (int i = 0; i < tokenCount; i++) {
//         Serial.print("Param ");
//         Serial.print(i + 1);
//         Serial.print(": ");
//         Serial.println(tokens[i]);
//     }
// }

// void setup() {
//     hardParser.begin();
//     hardParser.setCommandCallback(commandHandler);

//     Serial.println("HardSerialParser Test");
// }

// void loop() {
//     hardParser.processSerial();
// }
