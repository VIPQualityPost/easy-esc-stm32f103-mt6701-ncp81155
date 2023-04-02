/*
 * StreamTest - RTT stream test
 *
 * This program requires a "Segger" brand debugger probe to run.
 */

#include <Arduino.h>
#include <RTTStream.h>
RTTStream rtt;

void setup()
{
	rtt.blockUpBufferFull();
	rtt.setTimeout(300000);	/* 5 minute timeout for entry */

	/* stream output */
	rtt.println();
	rtt.println("rtt stream test");
	rtt.print("char: ");
	rtt.println('a');
	rtt.print("string: ");
	rtt.println("the quick brown fox jumps over a lazy dog");
	rtt.print("int: ");
	rtt.println(12345);
	rtt.print("float: ");
	rtt.println(3.1415926535);
	rtt.printf("printf char: %c signed int: %d unsigned int: %u\r\n", 'a', -12345, 12345);
	rtt.printf("printf hex int: %x string: %s pointer: %p\r\n", 0x12345, "12345", &rtt);

	/* stream input */
	rtt.print("enter an int: ");
	int i = rtt.parseInt();
	rtt.print("value: ");
	rtt.println(i);
	rtt.print("enter a float: ");
	float f = rtt.parseFloat();
	rtt.print("value: ");
	rtt.println(f);
	while (rtt.available()) rtt.read();
	rtt.print("enter a string and wait 5 seconds: ");
	rtt.setTimeout(5000);
	while (!rtt.available());
	String s = rtt.readString();
	rtt.println();
	rtt.print("string: ");
	rtt.println(s);
	rtt.println("rtt stream test end");
}

void loop()
{
}
/* not truncated */
