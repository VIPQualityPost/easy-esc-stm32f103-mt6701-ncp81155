/*
 * Echo - RTT echo test
 *
 * This program requires a "Segger" brand debugger probe to run.
 */

#include <RTTStream.h>
RTTStream rtt;

void setup()
{
	rtt.println("rtt echo test");
	rtt.println("type a character; the program will echo the next ascii character");
}

void loop()
{
	while (rtt.available())
		rtt.write(rtt.read() + 1);
}
/* not truncated */
