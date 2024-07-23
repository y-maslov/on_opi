import time
from opi_lora_spi_classes import SX1278, DoubleSX1278
import OPi.GPIO as GPIO


try:
    double_sx1278 = DoubleSX1278(
        rx_spi_bus=0, rx_spi_device=0, rx_reset_pin=8,
        tx_spi_bus=0, tx_spi_device=1, tx_reset_pin=10,
        gpio_a=33, gpio_b=29  # Пример GPIO пинов для управления мультиплексором
    )

    # Проверка подключения
    if not double_sx1278.check_connections():
        print("Error: Failed to establish connection with one or both SX1278 modules.")
        GPIO.cleanup()
        raise SystemExit
    else:
        print("Success: Both SX1278 modules are connected correctly.")

    # Настраиваем параметры приёмника
    double_sx1278.receiver.set_frequency(437.5)
    double_sx1278.receiver.set_bandwidth('250kHz')
    double_sx1278.receiver.set_spreading_factor(10)
    double_sx1278.receiver.set_low_data_rate_optimize(enabled=True)
    double_sx1278.receiver.set_sync_word(0x12)
    double_sx1278.receiver.set_preamble_length(8)
    double_sx1278.receiver.set_coding_rate(2)
    double_sx1278.receiver.set_tx_power(17)

    # Настраиваем параметры передатчика
    double_sx1278.transmitter.set_frequency(434.703)
    double_sx1278.transmitter.set_bandwidth('250kHz')
    double_sx1278.transmitter.set_spreading_factor(10)
    double_sx1278.transmitter.set_low_data_rate_optimize(enabled=True)
    double_sx1278.transmitter.set_sync_word(0x12)
    double_sx1278.transmitter.set_preamble_length(8)
    double_sx1278.transmitter.set_coding_rate(2)
    double_sx1278.transmitter.set_tx_power(17)

    # Получение конфигурации
    print("Receiver Configuration:")
    double_sx1278.get_receiver_configuration()

    print("Transmitter Configuration:")
    double_sx1278.get_transmitter_configuration()

    double_sx1278.receiver.set_mode_receive()
    while(True):
        double_sx1278.switch_to_receiver()
        print("reciever switch, RX_SX1278!")

        # Принимаем данные
        print("-" * 30)
        print(f"CURRENT RSSI: {double_sx1278.receiver.get_current_rssi()} dBm")
        data = double_sx1278.read_data()
        print(f"DATA: {data}")
        formatted_lines = []
        for i in range(0, len(data), 16):
            line = data[i:i+16]
            formatted_line = ' '.join(f'{byte:02X}' for byte in line)
            formatted_lines.append(formatted_line)

        # Вывод строк
        for line in formatted_lines:
            print(line)


        named_tuple = time.localtime() # get struct_time
        time_string = time.strftime("%m/%d/%Y, %H:%M:%S", named_tuple)
        print(f"time: {time_string}")
        print(f"Message Counter: {double_sx1278.receiver.message_counter}")
        
        if data != b'': 
            print("-" * 30)
            # Отправляем данные (передатчики, меняем перключатели)
            print("передатчики, меняем перключатели")
            double_sx1278.switch_to_transmitter()
            time.sleep(1)
            double_sx1278.send_data(data)
            print("transmitter switch, TX_SX1278!")




except KeyboardInterrupt:
    GPIO.cleanup()
    raise SystemExit