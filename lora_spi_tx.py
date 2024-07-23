import time
from opi_lora_spi_classes import SX1278
import OPi.GPIO as GPIO



try:
    # Пример использования класса
    sx1278 = SX1278(spi_bus=0, spi_device=0, cs_pin=24, reset_pin=10)
    connected = sx1278.check_connection()
    if connected:
        # Настраиваем параметры
        sx1278.set_frequency(436.7)
        sx1278.set_bandwidth('250kHz')
        sx1278.set_spreading_factor(10)
        #sx1278.calculate_low_data_rate_optimize('250kHz', 10)
        sx1278.set_low_data_rate_optimize(enabled=True)
        sx1278.set_sync_word(0x12)
        sx1278.set_preamble_length(8)
        sx1278.set_coding_rate(2)
        sx1278.set_tx_power(17)
    
        time.sleep(1)
        sx1278.get_configuration()

        while True:
            # Отправляем данные
            s0 = "As is summarized in Table 1, the present version of the KK MC has almost the full functionality of the older KORALZ and KORALB"
            print("-" * 30)
            print(f"sended {s0}")
            sx1278.send_data(s0.encode('utf-8'))
            time.sleep(3)

            s1 = "Hello world!"
            print("-" * 30)
            print(f"sended {s1}")
            sx1278.send_data(s1.encode('utf-8'))
            time.sleep(3)

            s2 = """Disclaimer and copyright notice The information in this article, including the URL address for reference,
                is subject to change without priornotice.The Documentation is provided "as is" without any warranty, including any warranties ofmerch"""
            print("-" * 30)
            print(f"sended {s2}")
            sx1278.send_data(s2.encode('utf-8'))
            time.sleep(3)
    else:
        print("Not connected")
except KeyboardInterrupt:
    GPIO.cleanup()
    raise SystemExit