from typing import Self
import machine

class LoRa:
    """A wrapper class for UART connected AT command driven LoRa modules"""

    class CommandError(Exception): 
        """An exception class for AT command errors"""

    class RecvData:
        """Users are not meant to instantiate this class, only use it to access received data."""

        def __init__(self, address: int, data: str, rssi: int, snr: int) -> None:
            self._address = address
            self._data = data
            self._rssi = rssi
            self._snr = snr

            self._exception = None
            self._raw_data = None
        
        @classmethod
        def _init_error(cls, exception: Exception, raw_data: bytes) -> Self:
            err = cls(-1, "", 0, 0)

            err._address = None
            err._data = None
            err._rssi = None
            err._snr = None

            err._exception = exception
            err._raw_data = raw_data

            return err
        
        def is_error(self) -> bool:
            """Returns whether the stored data represents an error (`true`) or data (`false`)"""

            return self._exception is not None \
                and self._raw_data is not None \
                and self._address is None \
                and self._data is None \
                and self._rssi is None \
                and self._snr is None
        
        def get_data(self) -> tuple[int, str, int, int]:
            """
            Returns stored data. It is **highly** recommended to check that the data is not an error before calling this.
            Example of checking for error::

                data = lora.recv() # `lora` is an instance of the `LoRa` class

                if (err := data.get_error()):
                    e, data = err

                else:
                    address, data, rssi, snr = data.get_data()
            
            ## Return
                returns a tuple containing the following:
                    - `address`: an `int` which represent the address the data is from
                    - `data`: a `str` holding the received data
                    - `rssi`: an `int` with the RSSI value
                    - `snr`: an `int` with the SNR value
            """
            
            return (self._address, self._data, self._rssi, self._snr)
        
        def get_error(self) -> None | tuple[Exception, bytes]:
            """Returns any stored error or `None` if there is no error"""

            if self._exception is not None \
                and self._raw_data is not None \
                and self._address is None \
                and self._data is None \
                and self._rssi is None \
                and self._snr is None:

                return (self._exception, self._raw_data)
        
    def __init__(self, tx_pin_num: int, rx_pin_num: int, baudrate: int = 115200) -> None:
        """
        Initialize the LoRa class and the internal UART.

        ### Parameters
            - `tx_pin_num`: the TX pin number
            - `rx_pin_num`: the RX pin number
            - `baudrate`: optional baudrate, default is `115200`
        """
        
        self._uart = machine.UART(0, baudrate = baudrate, tx = machine.Pin(tx_pin_num), rx = machine.Pin(rx_pin_num))

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

    def command(self, command: str, ignore_errors: bool = False) -> bytes:
        """
        Run an AT command on LoRa module, optionally ignoring errors.
        
        ### Parameters
            - `command`: the AT command to be executed on the module
            - `ignore_errors`: whether any returned errors should be ignored or raised
        
        ### Return
            Returns the module's response in bytes
        
        ### Raises
            - `CommandError`: if `ignore_errors` is true, and if an error AT command was received from the module
        """

        self._uart.write(f"{command}\r\n")

        while not self._uart.any():
            pass

        result = self._uart.read()

        if not ignore_errors and result.startswith(b"+ERR"):
            raise self.CommandError(f"`{command}` caused error \"{self._errors[int((x := result.decode().strip()).split("=")[1])]}\" (`{x}`)")
        
        return result
    
    def setup(self, network_id: int = 18, address: int = 0, spreading_factor: int = 9, bandwidth: int = 7, coding_rate: int = 1, programmed_preamble: int = 12) -> None:
        """
        Sets up the LoRa module, making it ready to use.
        based off of these docs: https://lemosint.com/wp-content/uploads/2021/11/Lora_AT_Command_RYLR998_RYLR498_EN.pdf
        
        ### Parameters
            - `network_id`: 3-15 + 18, This LoRa module's network ID.
            - `address`: 0-65535, This LoRa module's address, must be unique.
            - `spreading_factor`: 7-10, The larger the SF is, the better the sensitivity is, but the transmission time will take longer.
            - `bandwidth`: 7-9, The smaller the bandwidth is, the better the sensitivity is, but the transmission time will take longer.
            - `coding_rate`: 1-4, The coding rate will be the fastest if setting it as 1.
            - `programmed_preamble`: (Recommended to not change) Preamble code. If the preamble code is bigger, it will result in the less opportunity of losing data. Generally preamble code can be set above 10 if under the permission of the transmission time.
        
        ### Raises
            - `CommandError`: will propagate up from `command` method calls
        """

        self.command("AT", ignore_errors = True)
        self.command(f"AT+PARAMETER={spreading_factor},{bandwidth},{coding_rate},{programmed_preamble}")
        self.command(f"AT+ADDRESS={address}")
        self.command(f"AT+NETWORKID={network_id}")

    def reset(self) -> bytes:
        """Runs the AT command to reset LoRa module"""

        return self.command("AT+RESET")
    
    def send(self, address: int, data: str) -> bytes:
        """Runs send AT command on LoRa module, if `address` is 0, module will broadcast on all networks"""

        return self.command(f"AT+SEND={address},{len(data)},{data}")

    def recv_raw(self) -> bytes:
        """A blocking function that waits for data to be received, then returns the raw bytes"""

        while not self._uart.any():
            pass

        return self._uart.read()
        
    def recv(self) -> RecvData:
        """A blocking function that waits for data to be received, then returns the data"""

        raw = self.recv_raw()

        try:
            recv = raw.decode()[5:].split(",")

            return self.RecvData(int(recv[0]), recv[2], int(recv[3]), int(recv[4]))

        except Exception as e:
            return self.RecvData._init_error(e, raw)

    def query(self, name: str):
        """Query LoRa module for variable value"""

        return self.command(f"AT+{name}?").decode().strip().split("=")[1]

lora = LoRa(0, 1)
data = lora.recv()

if (err := data.get_error()):
    e, data = err

else:
    address, data, rssi, snr = data.get_data()