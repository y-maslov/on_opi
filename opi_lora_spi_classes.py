import spidev
import orangepi.pc
from OPi import GPIO
import time
import os

# The pin numerariton is "physical" (gpio readall medium)

class SX1278:
    """
    A class to manage communication with the SX1278 transceiver module on an Orange Pi PC.

    Attributes
    ----------
    spi : spidev.SpiDev
        SPI interface for communication with SX1278.
    cs_pin : int
        GPIO pin used for Chip Select (CS).
    reset_pin : int
        GPIO pin used for resetting the SX1278.
    dio0_pin : int, optional
        GPIO pin used for DIO0 (interrupt signal). If None, DIO0 is not used.

    Methods
    -------
    __init__(spi_bus, spi_device, cs_pin, reset_pin)
        Initializes the SPI interface and configures GPIO pins.
    reset()
        Resets the SX1278 module.
    write_register(address, value)
        Writes a value to a register on the SX1278.
    read_register(address)
        Reads a value from a register on the SX1278.
    configure()
        Configures the SX1278 module for transmission and reception.
    check_connection()
        Checks if the SX1278 is correctly connected and communication is established.
    set_mode_receive()
        Configures the SX1278 to receive mode.
    set_mode_transmit()
        Configures the SX1278 to transmit mode.
    send_data(data)
        Sends data using the SX1278 module.
    read_data()
        Reads data received by the SX1278 module.
    set_frequency(freq)
        Sets the operating frequency.
    set_tx_power(level)
        Sets the transmission power level.
    set_bandwidth(bandwidth)
        Sets the bandwidth.
    set_spreading_factor(sf)
        Sets the spreading factor.
    set_low_data_rate_optimize(enabled)
        Sets the Low Data Rate Optimize parameter.
    calculate_low_data_rate_optimize(bandwidth, spreading_factor)
        Calculates if Low Data Rate Optimize should be enabled based on bandwidth and spreading factor.
    set_sync_word(sync_word)
        Sets the sync word.
    set_preamble_length(preamble_length)
        Sets the preamble length.
    set_coding_rate(coding_rate)
        Sets the coding rate.
    get_configuration()
        Retrieves the current configuration of the SX1278 and prints it in a human-readable format.
    """

    def __init__(self, spi_bus, spi_device, cs_pin, reset_pin):
        """
        Initializes the SPI interface and configures GPIO pins.

        Parameters
        ----------
        message_counter: int
            message counter.
        spi_bus : int
            SPI bus number.
        spi_device : int
            SPI device number.
        cs_pin : int
            GPIO pin used for Chip Select (CS).
        reset_pin : int
            GPIO pin used for resetting the SX1278.
        """
        self.message_counter = 0  # Initialize message counter

        self.spi = spidev.SpiDev()
        try:
            self.spi.open(spi_bus, spi_device)
        except PermissionError:
            raise RuntimeError("Permission denied: Try running the script with 'sudo'")

        self.spi.max_speed_hz = 5000000

        # self.cs_pin = cs_pin
        self.reset_pin = reset_pin

        GPIO.setmode(orangepi.pc.BOARD)
        # GPIO.setup(self.cs_pin, GPIO.OUT)
        GPIO.setup(self.reset_pin, GPIO.OUT)

        self.reset()
        self.configure()




    def reset(self):
        """
        Resets the SX1278 module by toggling the reset pin.
        """
        GPIO.output(self.reset_pin, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self.reset_pin, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(self.reset_pin, GPIO.HIGH)
        time.sleep(0.1)

    def write_register(self, address, value):
        """
        Writes a value to a register on the SX1278.

        Parameters
        ----------
        address : int
            The register address.
        value : int
            The value to write to the register.
        """
        # GPIO.output(self.cs_pin, GPIO.LOW)
        self.spi.xfer2([address | 0x80, value])
        # GPIO.output(self.cs_pin, GPIO.HIGH)

    def read_register(self, address):
        """
        Reads a value from a register on the SX1278.

        Parameters
        ----------
        address : int
            The register address.

        Returns
        -------
        int
            The value read from the register.
        """
        # GPIO.output(self.cs_pin, GPIO.LOW)
        value = self.spi.xfer2([address & 0x7F, 0x00])
        # GPIO.output(self.cs_pin, GPIO.HIGH)
        return value[1]

    def configure(self):
        """
        Configures the SX1278 module for transmission and reception.
        """
        self.write_register(0x01, 0x80)  # Enter sleep mode
        time.sleep(0.1)
        self.write_register(0x01, 0x81)  # Set to LoRa mode
        self.write_register(0x1D, 0x72)  # Configure bandwidth, coding rate, etc.
        self.write_register(0x1E, 0x74)  # Configure spreading factor, etc.
        self.write_register(0x26, 0x04)  # Configure low data rate optimization, etc.
        self.write_register(0x39, 0x34)  # Configure sync word, etc.

    def check_connection(self):
        """
        Checks if the SX1278 is correctly connected and communication is established.

        Returns
        -------
        bool
            True if the SX1278 is correctly connected, False otherwise.
        """
        version = self.read_register(0x42)
        if version == 0x12:  # Assuming 0x12 is the expected version value for SX1278
            return True
        return False

    def set_mode_receive(self):
        """
        Configures the SX1278 to receive mode.
        """
        self.write_register(0x01, 0x81)  # Set to standby mode
        self.write_register(0x40, 0x00)  # Set DIO0 to RX done
        self.write_register(0x01, 0x8D)  # Set to continuous RX mode

    def set_mode_transmit(self):
        """
        Configures the SX1278 to transmit mode.
        """
        self.write_register(0x01, 0x81)  # Set to standby mode
        self.write_register(0x40, 0x40)  # Set DIO0 to TX done

    def send_data(self, data):
        """
        Sends data using the SX1278 module.

        Parameters
        ----------
        data : bytes
            The data to be sent.
        """
        self.set_mode_transmit()
        self.write_register(0x0E, 0x00)  # FIFO TX base address
        self.write_register(0x0D, 0x00)  # FIFO pointer
        self.write_register(0x22, len(data))  # Payload length
        for byte in data:
            self.write_register(0x00, byte)
            #print(byte)
        
        self.write_register(0x01, 0x8B)  # Set to TX mode
        time.sleep(1)  # Wait for a reasonable amount of time for transmission to complete


        irq_flags = self.read_register(0x12)
        while not irq_flags & 0x08:
            time.sleep(1)
            irq_flags = self.read_register(0x12)
        self.write_register(0x12, 0xFF)  # Clear all IRQ flags

    def read_data(self):
        """
        Reads data received by the SX1278 module.

        Returns
        -------
        bytes
            The data received.
        """
        
        time.sleep(1)  # Wait for a reasonable amount of time for reception to complete
        irq_flags = self.read_register(0x12)

        
        if not irq_flags & 0x40:  # Check for RX_DONE flag
            print("RX_DONE not set (no new packages)")
            self.write_register(0x12, 0xFF)  # Clear all IRQ flags
            return bytes(0)


        if irq_flags & 0x20:  # Check for payload CRC error
            self.write_register(0x12, 0x20)  # Clear IRQ flags
            print("CRC error")
            # return None


        payload_length = self.read_register(0x13)
        self.write_register(0x0D, 0x00)  # FIFO pointer
        data = bytes([self.read_register(0x00) for _ in range(payload_length)])
        self.write_register(0x12, 0xFF)  # Clear IRQ flags
    

        # Increment message counter
        self.message_counter += 1

        # Retrieve RSSI and SNR
        rssi = self.get_rssi()
        snr = self.get_snr()
        
        # Print RSSI and SNR values
        print(f"PACKAGE RSSI: {rssi} dBm")
        print(f"SNR: {snr} dB")
        
        # Clear Fifo and set rx continious again
        self.set_mode_receive()
        
        return data

    def get_rssi(self):
        """
        Retrieves the last package RSSI value.

        Returns
        -------
        float
            The RSSI value in dBm.
        """
        packet_rssi = self.read_register(0x1A)
      

        last_pkg_rssi = packet_rssi - 164
        snr = self.get_snr()
        if snr < 0.0: 
            last_pkg_rssi += snr
        return last_pkg_rssi
    
    def get_current_rssi(self):
        """
        Retrieves the current RSSI value.

        Returns
        -------
        float
            The RSSI value in dBm.
        """
        rssi = self.read_register(0x1B)
        return rssi - 164 # YM: использую low freq mode

    def get_snr(self):
        """
        Retrieves the current SNR value.

        Returns
        -------
        float
            The SNR value in dB.
        """
        snr = self.read_register(0x19)
        if snr & 0x80:  # SNR is negative
            snr = (snr & 0x7F) - 128
        return snr / 4.0

    def set_frequency(self, freq):
        """
        Sets the operating frequency.

        Parameters
        ----------
        freq : float
            Frequency in MHz.
        """
        frf = int((freq * 1000000.0) / 61.03515625)
        self.write_register(0x06, (frf >> 16) & 0xFF)
        self.write_register(0x07, (frf >> 8) & 0xFF)
        self.write_register(0x08, frf & 0xFF)

    def set_tx_power(self, level):
        """
        Sets the transmission power level.

        Parameters
        ----------
        level : int
            Power level in dBm (2-17).
        """
        # YM: i assume that we use PA_BOOST pin
        if level == 20:
            self.write_register(0x09, 0xFF)
            self.write_register(0x4D, 0x07)
            return 
        elif level < 2:
            level = 2
        elif level > 17:
            level = 17
        output_power = level - 2
        self.write_register(0x09, (0xF0 | output_power))
        self.write_register(0x4D, 0x04)

    def set_bandwidth(self, bandwidth):
        """
        Sets the bandwidth.

        Parameters
        ----------
        bandwidth : str
            Bandwidth as a string ('7.8kHz', '10.4kHz', '15.6kHz', '20.8kHz', '31.25kHz', '41.7kHz', '62.5kHz', '125kHz', '250kHz', '500kHz').
        """
        bw_dict = {
            '7.8kHz': 0x00,
            '10.4kHz': 0x01,
            '15.6kHz': 0x02,
            '20.8kHz': 0x03,
            '31.25kHz': 0x04,
            '41.7kHz': 0x05,
            '62.5kHz': 0x06,
            '125kHz': 0x07,
            '250kHz': 0x08,
            '500kHz': 0x09
        }
        if bandwidth in bw_dict:
            reg_value = self.read_register(0x1D)
            reg_value = (reg_value & 0x0F) | (bw_dict[bandwidth] << 4)
            self.write_register(0x1D, reg_value)

    def set_spreading_factor(self, sf):
        """
        Sets the spreading factor.

        Parameters
        ----------
        sf : int
            Spreading factor (6-12).
        """
        if sf < 6:
            sf = 6
        elif sf > 12:
            sf = 12
        self.write_register(0x1E, (self.read_register(0x1E) & 0x0F) | ((sf << 4) & 0xF0))
        if sf == 6:
            self.write_register(0x31, 0xC5)
            self.write_register(0x37, 0x0C)
        else:
            self.write_register(0x31, 0xC3)
            self.write_register(0x37, 0x0A)

    def set_low_data_rate_optimize(self, enabled):
        """
        Sets the Low Data Rate Optimize parameter.

        Parameters
        ----------
        enabled : bool
            If True, enables Low Data Rate Optimize. If False, disables it.
        """
        current_value = self.read_register(0x26)
        if enabled:
            self.write_register(0x26, current_value | 0x08)
        else:
            self.write_register(0x26, current_value & 0xF7)

    def calculate_low_data_rate_optimize(self, bandwidth, spreading_factor):
        """
        Calculates if Low Data Rate Optimize should be enabled based on bandwidth and spreading factor.

        Parameters
        ----------
        bandwidth : str
            Bandwidth as a string.
        spreading_factor : int
            Spreading factor (6-12).
        """
        bw_dict = {
            '7.8kHz': 7800,
            '10.4kHz': 10400,
            '15.6kHz': 15600,
            '20.8kHz': 20800,
            '31.25kHz': 31250,
            '41.7kHz': 41700,
            '62.5kHz': 62500,
            '125kHz': 125000,
            '250kHz': 250000,
            '500kHz': 500000
        }
        if bandwidth in bw_dict:
            bw = bw_dict[bandwidth]
            t_sym = (2 ** spreading_factor) / bw
            if t_sym > 0.016:
                self.set_low_data_rate_optimize(True)
            else:
                self.set_low_data_rate_optimize(False)

    def set_sync_word(self, sync_word):
        """
        Sets the sync word.

        Parameters
        ----------
        sync_word : int
            The sync word (1 byte).
        """
        self.write_register(0x39, sync_word)

    def set_preamble_length(self, preamble_length):
        """
        Sets the preamble length.

        Parameters
        ----------
        preamble_length : int
            The preamble length.
        """
        self.write_register(0x20, (preamble_length >> 8) & 0xFF)
        self.write_register(0x21, preamble_length & 0xFF)

    def set_coding_rate(self, coding_rate):
        """
        Sets the coding rate.

        Parameters
        ----------
        coding_rate : int
            The coding rate (1-4).
        """
        if coding_rate < 1:
            coding_rate = 1
        elif coding_rate > 4:
            coding_rate = 4
        current_value = self.read_register(0x1D)
        self.write_register(0x1D, (current_value & 0xF1) | ((coding_rate - 1) << 1))

    def get_configuration(self):
        """
        Retrieves the current configuration of the SX1278 and prints it in a human-readable format.
        """
        frf = (self.read_register(0x06) << 16) | (self.read_register(0x07) << 8) | self.read_register(0x08)
        frequency = frf * 61.03515625 / 1000000.0
        regPaDac_2_0 = self.read_register(0x4D) & 0x07
        tx_power = self.read_register(0x09) & 0x0F

        if regPaDac_2_0 == 0x07 and tx_power == 0x0F: 
            tx_power = 20
        else:
            tx_power = tx_power + 2 if tx_power < 15 else 17

        bandwidth = (self.read_register(0x1D) & 0xF0) >> 4
        bandwidth_dict = {
            0x00: '7.8kHz',
            0x01: '10.4kHz',
            0x02: '15.6kHz',
            0x03: '20.8kHz',
            0x04: '31.25kHz',
            0x05: '41.7kHz',
            0x06: '62.5kHz',
            0x07: '125kHz',
            0x08: '250kHz',
            0x09: '500kHz'
        }
        bandwidth = bandwidth_dict.get(bandwidth, 'Unknown')

        spreading_factor = (self.read_register(0x1E) & 0xF0) >> 4

        low_data_rate_optimize = (self.read_register(0x26) & 0x08) >> 3

        sync_word = self.read_register(0x39)

        preamble_length = (self.read_register(0x20) << 8) | self.read_register(0x21)

        coding_rate = (self.read_register(0x1D) & 0x0E) >> 1

        print(f"SX1278 Configuration:")
        print(f"Frequency: {frequency} MHz")
        print(f"TX Power: {tx_power} dBm")
        print(f"Bandwidth: {bandwidth}")
        print(f"Spreading Factor: SF{spreading_factor}")
        print(f"Low Data Rate Optimize: {'Enabled' if low_data_rate_optimize else 'Disabled'}")
        print(f"Sync Word: 0x{sync_word:02X}")
        print(f"Preamble Length: {preamble_length}")
        print(f"Coding Rate: 4/{coding_rate + 4}")


class DoubleSX1278:
    """
    A class to manage two SX1278 transceiver modules on an Orange Pi PC,
    with one always configured for receiving and the other for transmitting,
    and using an HMC435 multiplexer to switch between a single antenna.

    Attributes
    ----------
    receiver : SX1278
        SX1278 module configured for receiving.
    transmitter : SX1278
        SX1278 module configured for transmitting.
    gpio_a : int
        GPIO pin used for control line A of the HMC435 multiplexer.
    gpio_b : int
        GPIO pin used for control line B of the HMC435 multiplexer.
    current_state : str
        Current state of the multiplexer ('receiver' or 'transmitter').

    Methods
    -------
    __init__(rx_spi_bus, rx_spi_device, rx_cs_pin, rx_reset_pin,
             tx_spi_bus, tx_spi_device, tx_cs_pin, tx_reset_pin,
             gpio_a, gpio_b)
        Initializes the receiver and transmitter modules and configures the GPIO pins for the multiplexer.
    send_data(data)
        Sends data using the transmitter module.
    read_data()
        Reads data received by the receiver module.
    check_connections()
        Checks if both the receiver and transmitter SX1278 modules are correctly connected.
    get_receiver_configuration()
        Retrieves the current configuration of the receiver SX1278 and prints it in a human-readable format.
    get_transmitter_configuration()
        Retrieves the current configuration of the transmitter SX1278 and prints it in a human-readable format.
    switch_to_receiver()
        Switches the multiplexer to connect the receiver module.
    switch_to_transmitter()
        Switches the multiplexer to connect the transmitter module.
    """

    def __init__(self, rx_spi_bus, rx_spi_device, rx_cs_pin, rx_reset_pin,
                 tx_spi_bus, tx_spi_device, tx_cs_pin, tx_reset_pin,
                 gpio_a, gpio_b):
        """
        Initializes the receiver and transmitter modules and configures the GPIO pins for the multiplexer.

        Parameters
        ----------
        rx_spi_bus : int
            SPI bus number for the receiver.
        rx_spi_device : int
            SPI device number for the receiver.
        rx_cs_pin : int
            GPIO pin used for Chip Select (CS) on the receiver.
        rx_reset_pin : int
            GPIO pin used for resetting the receiver.
        tx_spi_bus : int
            SPI bus number for the transmitter.
        tx_spi_device : int
            SPI device number for the transmitter.
        tx_cs_pin : int
            GPIO pin used for Chip Select (CS) on the transmitter.
        tx_reset_pin : int
            GPIO pin used for resetting the transmitter.
        gpio_a : int
            GPIO pin used for control line A of the HMC435 multiplexer.
        gpio_b : int
            GPIO pin used for control line B of the HMC435 multiplexer.
        """
        self.receiver = SX1278(rx_spi_bus, rx_spi_device, rx_cs_pin, rx_reset_pin)
        self.transmitter = SX1278(tx_spi_bus, tx_spi_device, tx_cs_pin, tx_reset_pin)

        self.gpio_a = gpio_a
        self.gpio_b = gpio_b
        self.current_state = 'transmitter'

        print(GPIO.setmode(orangepi.pc.BOARD))
        GPIO.setup(self.gpio_a, GPIO.OUT)
        GPIO.setup(self.gpio_b, GPIO.OUT)

        self.receiver.set_mode_receive()
        self.switch_to_receiver() # YM: reciever is out1?

    def send_data(self, data):
        """
        Sends data using the transmitter module.

        Parameters
        ----------
        data : bytes
            The data to be sent.
        """
        self.switch_to_transmitter()
        self.transmitter.send_data(data)
        self.switch_to_receiver()

    def read_data(self):
        """
        Reads data received by the receiver module.

        Returns
        -------
        bytes
            The data received.
        """
        self.switch_to_receiver()
        return self.receiver.read_data()

    def check_connections(self):
        """
        Checks if both the receiver and transmitter SX1278 modules are correctly connected.

        Returns
        -------
        bool
            True if both modules are correctly connected, False otherwise.
        """
        return self.receiver.check_connection() and self.transmitter.check_connection()

    def get_receiver_configuration(self):
        """
        Retrieves the current configuration of the receiver SX1278 and prints it in a human-readable format.
        """
        self.receiver.get_configuration()

    def get_transmitter_configuration(self):
        """
        Retrieves the current configuration of the transmitter SX1278 and prints it in a human-readable format.
        """
        self.transmitter.get_configuration()

    def switch_to_receiver(self):
        """
        Switches the multiplexer to connect the receiver module. 
        """
        if self.current_state != 'receiver':
            GPIO.output(self.gpio_a, GPIO.LOW)
            GPIO.output(self.gpio_b, GPIO.HIGH)
            print(f"a ({self.gpio_a}) - low, b ({self.gpio_b}) - high")
            self.current_state = 'receiver'

    def switch_to_transmitter(self):
        """
        Switches the multiplexer to connect the transmitter module.
        """
        if self.current_state != 'transmitter':
            GPIO.output(self.gpio_a, GPIO.HIGH)
            GPIO.output(self.gpio_b, GPIO.LOW)
            print(f"a ({self.gpio_a}) - high, b ({self.gpio_b}) - low")
            self.current_state = 'transmitter'


class Tee:
    """
    Класс, который будет записывать данные в стандартный поток (консоль) и в файл
    # Пример использования
    with open('output.txt', 'w') as f:
        tee = Tee(sys.stdout, f)
        sys.stdout = tee

        # Теперь все, что выводится с помощью print, пойдет и в консоль, и в файл
        print("This will be printed to the console and written to the file.")
        print("Another line.")

    # Восстанавливаем стандартный поток вывода
    sys.stdout = sys.__stdout__
    """


    def __init__(self, *files):
        self.files = files

    def write(self, obj):
        for f in self.files:
            f.write(obj)
            f.flush()  # обеспечиваем немедленную запись

    def flush(self):
        for f in self.files:
            f.flush()

