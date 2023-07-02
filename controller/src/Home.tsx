import { useEffect, useRef, useState } from "react";

const baudRate = 115200;

export const Home = () => {
  const [ports, setPorts] = useState<SerialPort[]>([]);
  const [connectedPort, setConnectedPort] = useState<SerialPort>(null);
  const [serialOutput, setSerialOutput] = useState<string>("");
  const reader = useRef<ReadableStreamDefaultReader<string>>();
  const readableStreamClosed = useRef<Promise<void>>();

  const changePort = async (port?: SerialPort) => {
    if (connectedPort) {
      try {
        reader.current.cancel();
        await readableStreamClosed.current.catch(() => {
          /* Ignore the error */
        });
        await connectedPort.close();
        setConnectedPort(null);
        console.log("closed port");
      } catch (e) {
        console.log(e);
      }
    }

    if (port) {
      await port.open({
        baudRate,
      });
      console.log("opened port");
      setConnectedPort(port);

      const decoder = new TextDecoderStream();
      readableStreamClosed.current = port.readable.pipeTo(decoder.writable);
      reader.current = decoder.readable.getReader();
      while (true) {
        const { value, done } = await reader.current.read();
        if (value) {
          setSerialOutput((serialOutput) => serialOutput + value);
        }
        if (done) {
          console.log("[readLoop] DONE", done);
          reader.current.releaseLock();
          break;
        }
      }
    }
  };

  useEffect(() => {
    navigator.serial.getPorts().then((ports) => setPorts(ports));
  }, []);

  return (
    <>
      <h2>Controller</h2>
      Ports:{" "}
      <select
        placeholder="Select Port"
        onChange={async (e) => {
          const selectedIndex = parseInt(e.target.value);
          changePort(ports[selectedIndex]);
        }}
      >
        <option value={-1}>Select a port</option>
        {ports.map((port, i) => (
          <option key={"port_" + i} value={i}>
            {port.getInfo().usbVendorId} - {port.getInfo().usbProductId}
          </option>
        ))}
      </select>
      <div>
        <h3>Output</h3>
        {serialOutput.split("\n").map((line) => (
          <div>
            <code>{line}</code>
          </div>
        ))}
      </div>
    </>
  );
};
