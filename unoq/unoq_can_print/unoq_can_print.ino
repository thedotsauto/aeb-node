/*
  unoq_can_print.ino — Arduino UNO Q STM32U585 sketch, UNO Q mode only.

  Receives CAN frames relayed by can_bridge.py (running on the UNO Q's Linux
  MPU) over the Arduino UNO Q Bridge, and prints them to the Monitor.

  This is stage 1 of UNO Q support: the STM32 only PRINTS the frame it
  receives. It does not transmit on MCP2515 and the physical CAN bus is not
  connected. MCP2515 initialization/transmission is a later phase and is
  intentionally not added here.

  Bridge API (Arduino_RouterBridge, "designed for Arduino UNO Q boards" per
  its own README):

      Bridge.begin();
      Bridge.provide(method_name, handler);   // MCU exposes a callback the
                                               // Linux MPU side can invoke

  can_bridge.py calls, for every AEB CAN frame:

      Bridge.notify("can_frame", can_id, dlc, b0, b1, ..., b7)

  i.e. always exactly 10 parameters (dlc-8 unused trailing bytes are sent as
  0), so the handler below binds a fixed signature rather than a variable-
  length one.
*/

#include <Arduino_RouterBridge.h>

void printCanFrame(int can_id, int dlc, int b0, int b1, int b2, int b3,
                   int b4, int b5, int b6, int b7) {
    const int data[8] = {b0, b1, b2, b3, b4, b5, b6, b7};
    const int count = (dlc < 0) ? 0 : (dlc > 8 ? 8 : dlc);

    Monitor.println("================================");
    Monitor.println("CAN FRAME RECEIVED");
    Monitor.println("================================");
    Monitor.print("ID   : 0x");
    Monitor.println(can_id, HEX);
    Monitor.print("DLC  : ");
    Monitor.println(dlc);
    Monitor.print("DATA :");
    for (int i = 0; i < count; ++i) {
        Monitor.print(' ');
        if (data[i] < 16) {
            Monitor.print('0');
        }
        Monitor.print(data[i], HEX);
    }
    Monitor.println();
    Monitor.println("================================");
}

void setup() {
    Bridge.begin();
    Monitor.begin(115200);
    while (!Monitor) {}

    Bridge.provide("can_frame", printCanFrame);

    Monitor.println("unoq_can_print: ready, waiting for CAN frames");
}

void loop() {
    // Bridge.provide handlers are serviced on their own thread; nothing to
    // do here.
}
