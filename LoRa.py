import machine, utime
from .rs_result import Result, Ok, Err, Check

class LoRa:
    """A wrapper class for UART connected AT command driven LoRa modules"""

    class CommandError(Exception): 
        """An exception class for AT command errors"""
    
    class UARTTimeoutError(Exception):
        """An exception class for command timeouts"""
    
    class EmptyRecvError(Exception):
        """An exception class for empty received data"""

    class RecvData:
        """Users are not meant to instantiate this class, only use it to access received data."""

        AddressInt = int
        DataBytes = bytes
        RSSI_Int = int
        SNR_Int = int

        def __init__(self, address: int, data: bytes, rssi: int, snr: int) -> None:
            self._address = address
            self._data = data
            self._rssi = rssi
            self._snr = snr

        def unpack(self) -> tuple[AddressInt, DataBytes, RSSI_Int, SNR_Int]:
            """
            Unpacks all internal data.
            
            ### Returns
                - `AddressInt`: An `int` with the the received address value
                - `DataBytes`: A `bytes` object with the received data
                - `RSSI_Int`: An `int` with the signal RSSI
                - `SNR_Int`: An `int` with the signal SNR
            """

            return self.address, self.data, self.rssi, self.snr

        @property
        def address(self):
            return self._address
        
        @property
        def data(self):
            return self._data
        
        @property
        def rssi(self):
            return self._rssi
        
        @property
        def snr(self):
            return self._snr

    class RecvErr:
        """Users are not meant to instantiate this class, only use it to access received data."""

        RawDataBytes = bytes

        def __init__(self, exception: Exception, raw_data: bytes) -> None:
            self._exception = exception
            self._raw_data = raw_data

        def unpack(self) -> tuple[Exception, RawDataBytes]:
            """
            Unpacks all internal data.
            
            ### Returns
                - `Exception`: The caught error
                - `RawDataBytes`: A `bytes` object with the received data, unprocessed
            """

            return self.exception, self.raw_data
        
        @property
        def exception(self):
            return self._exception
        
        @property
        def raw_data(self):
            return self._raw_data
        
    def __init__(self, tx_pin_num: int, rx_pin_num: int, port: int = 0, baudrate: int = 115200) -> None:
        """
        Initialize the LoRa class and the internal UART.

        ### Parameters
            - `tx_pin_num`: the TX pin number
            - `rx_pin_num`: the RX pin number
            - `port`: required for UART init, default is 0
            - `baudrate`: optional baudrate, default is `115200`
        """
        
        self._uart = machine.UART(port, baudrate = baudrate, tx = machine.Pin(tx_pin_num), rx = machine.Pin(rx_pin_num))

        self._recv_buf: list[bytes] = []

        self._errors = {
            1: "There is not \"enter\" or 0x0D 0x0A in the end of the AT Command.",
            2: "The head of AT command is not \"AT\" string.",
            4: "Unknown command.",
            5: "The data to be sent does not match the actual length.",
            10: "TX is over times.",
            12: "CRC error.",
            13: "TX data exceeds 240 bytes.",
            14: "Failed to write flash memory.",
            15: "Unknown failure.",
            17: "Last TX was not completed.",
            18: "Preamble value is not allowed.",
            19: "RX failed, Header error.",
            20: "The time setting value of the \"Smart receiving power saving mode\" is not allowed."
        }

    def command(self, command: str, ignore_errors: bool = False, timeout: int = 0) -> Result[bytes, Exception]:
        """
        Run an AT command on LoRa module, optionally ignoring errors.
        
        ### Parameters
            - `command`: A `str` with the AT command to be executed on the module
            - `ignore_errors`: A `bool`, whether any LoRa module errors should be ignored or returned as `Err(str)`
        
        ### Returns
            a `Result` with `bytes` as `Ok` type and `str` as `Err` type
        """

        self._uart.write(f"{command}{chr(0x0D) + chr(0x0A)}".encode())

        result = self.read_raw(timeout = timeout)

        if Check.is_ok(result):
            raw = result.ok()
            
            for part in raw.split(b"\r\n"):                                                              # TODO NEEDS FURTHER TESTING
                if part.startswith(b"+RCV="):
                    self._recv_buf.append(part)

                elif part:
                    raw = part

            if not ignore_errors and raw.startswith(b"+ERR"):
                return Err(self.CommandError(f"`{command}` caused error \"{self._errors[int((x := raw.decode().strip()).split("=")[1])]}\" (`{x}`)"))
            
            return Ok(raw)

        if Check.is_err(result):
            return result.propagate()
        
        return Ok(b"")
    
    def setup(self, network_id: int = 18, address: int = 0, spreading_factor: int = 9, bandwidth: int = 7, coding_rate: int = 1, programmed_preamble: int = 12) -> Result[None, Exception]:
        """
        Sets up the LoRa module, making it ready to use.
        based off of these docs: https://lemosint.com/wp-content/uploads/2021/11/Lora_AT_Command_RYLR998_RYLR498_EN.pdf
        
        ### Parameters
            - `network_id`: `3-15 + 18`, this LoRa module's network ID
            - `address`: 0-65535, this LoRa module's address, must be unique
            - `spreading_factor`: `7-10`, the larger the SF is, the better the sensitivity is, but the transmission time will take longer
            - `bandwidth`: `7-9`, the smaller the bandwidth is, the better the sensitivity is, but the transmission time will take longer
            - `coding_rate`: `1-4`, the coding rate will be the fastest if setting it as 1
            - `programmed_preamble`: Preamble code. If the preamble code is bigger, it will result in the less opportunity of losing data. Generally preamble code can be set above 10 if under the permission of the transmission time
        
        ### Returns
            a `Result` with `None` as `Ok` type and `str` as `Err` type
        """
        
        rets: list[Result[bytes, Exception]] = []

        rets.append(self.command("AT", ignore_errors = True))
        rets.append(self.command(f"AT+PARAMETER={spreading_factor},{bandwidth},{coding_rate},{programmed_preamble}"))
        rets.append(self.command(f"AT+ADDRESS={address}"))
        rets.append(self.command(f"AT+NETWORKID={network_id}"))

        if (err := Check.first_err(rets)) is not None:
            return err.propagate()      
        
        return Ok(None)

    def reset(self) -> Result[bytes, Exception]:
        """
        Runs the AT command to reset LoRa module
        
        ### Returns
            a `Result` from the `LoRa.command` method call
        """

        return self.command("AT+RESET")
    
    def send(self, address: int, data: str) -> Result[bytes, Exception]:
        """
        Runs the send AT command on LoRa module, if `address` is 0, module will broadcast on all networks.
        
        ### Returns
            a `Result` from the `LoRa.command` method call
        """

        return self.command(f"AT+SEND={address},{len(data)},{data}")

    def read_raw(self, timeout: int = 0) -> Result[bytes, Exception]:
        """
        A blocking function that reads data from UART, capturing raw bytes.

        ### Parameters
            - `timeout`: An `int` with a default of 0 (no timeout)

        ### Returns
            a `Result` with `bytes` as `Ok` type and `Exception` as `Err` type
        """

        start_time = utime.time()
        
        try:
            while not self._uart.any():
                if timeout != 0 and (utime.time() - start_time) >= timeout:
                    return Err(self.UARTTimeoutError(f"`read_raw` exceeded the timeout value of {timeout} seconds"))

            return Ok(self._uart.read())
        
        except Exception as e:
            return Err(e)
        
    def recv(self, timeout: int = 0) -> Result[RecvData, RecvErr]:
        """
        A blocking function that waits for data to be received, parsed and wrapped in helper a class.

        ### Parameters
            - `timeout`: An `int` with a default of 0 (no timeout)
        
        ## Return
            a `Result` with `RecvData` as `Ok` type and `RecvErr` as `Err` type
        """

        result = self.read_raw(timeout = timeout)

        if Check.is_ok(result):           
            for part in result.ok().split(b"\r\n"):
                if part.startswith(b"+RCV="):
                    self._recv_buf.append(part)
                
                elif part:
                    return Err(self.RecvErr(self.EmptyRecvError("non +RCV data"), part))

        if Check.is_err(result):
            return Err(self.RecvErr(result.err(), b""))

        if not self._recv_buf:
            return Err(self.RecvErr(self.EmptyRecvError("recv buffer is empty"), b""))

        item = self._recv_buf.pop(0)

        try:
            recv = item[5:].split(b",")

            return Ok(self.RecvData(int(recv[0]), recv[2], int(recv[3]), int(recv[4])))

        except Exception as e:
            return Err(self.RecvErr(e, item))

    def query(self, name: str) -> Result[bytes, Exception]:
        """Query LoRa module for variable value."""

        result = self.command(f"AT+{name}?")

        # Removed due to MicroPython not supporting match-case :(
        # match result:
        #     case Ok():
        #         return Ok(result.ok().strip().split(b"=")[1])
            
        #     case Err():
        #         return result.propagate()

        if Check.is_ok(result):
            return Ok(result.ok().strip().split(b"=")[1])
            
        if Check.is_err(result):
            return result.propagate()

        return Ok(b"")