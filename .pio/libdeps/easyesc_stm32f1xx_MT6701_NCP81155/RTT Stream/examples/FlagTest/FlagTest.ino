/*
 * FlagTest - test RTT flags for data from host to target.
 *
 * Sample test:
 * - configure with rtt.blockDownBufferFull();
 * - paste 1000 characters in window
 * - check counter increases by 1000
 *
 * This program requires a "Segger" brand debugger probe to run.
 */

#include <RTTStream.h>

RTTStream rtt;

int count = 0;

void setup() {
  // uncomment one of the following three lines:
  rtt.blockDownBufferFull();
  //rtt.skipDownBufferFull();
  //rtt.trimDownBufferFull();

  rtt.println("start typing or paste text in this window");
}

void loop() {
  if (rtt.available()) {
    while (rtt.available()) {
      rtt.write(rtt.read());
      count++;
    }
    rtt.println();
    rtt.print(count);
    rtt.print(" ");
  }
}
