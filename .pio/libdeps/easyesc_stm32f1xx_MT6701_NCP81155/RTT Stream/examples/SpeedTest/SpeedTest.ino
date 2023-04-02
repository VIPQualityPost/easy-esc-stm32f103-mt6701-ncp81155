/*
 * SpeedTest - RTT speed test
 *
 * This program requires a "Segger" brand debugger probe to run.
 */

#include <Arduino.h>
#include <RTTStream.h>
RTTStream rtt;
const int count = 5000;


void setup()
{
	rtt.println();
	rtt.println("rtt speed test");
	rtt.blockUpBufferFull();

	unsigned long startTime = millis();
	unsigned long currentTime = millis();
	unsigned long overhead = currentTime - startTime;

	String str;
	for (char ch = 0; ch < 255; ch++)
		if (isPrintable(ch))
			str += ch;

	startTime = millis();
	for (int i = 0; i < count; i++)
		rtt.print(str);
	currentTime = millis();
	unsigned long t = currentTime - startTime - overhead;
	long speed = count * str.length() * 1000 / t;
	rtt.println();
	rtt.println("rtt speed test");
	rtt.print("speed: ");
	rtt.print(speed);
	rtt.println(" chars/s");
}

void loop()
{
}
/* not truncated */
