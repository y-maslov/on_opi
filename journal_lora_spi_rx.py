import time
import sys
import os
from opi_lora_spi_classes import SX1278, Tee
import OPi.GPIO as GPIO

try:
    # Получить текущее время запуска скрипта
    start_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())


    log_dir = "log"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    log_file_path = os.path.join(log_dir, f"{start_time}.txt")

    log_file = open(log_file_path, "a")
    tee = Tee(sys.stdout, log_file)
    sys.stdout = tee




    # Пример использования класса
    sx1278 = SX1278(spi_bus=0, spi_device=1, cs_pin=16, reset_pin=22)
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

        # # Отправляем данные
        # print("SEND DATA")
        # sx1278.send_data(b'Hello, SX1278!')
        time.sleep(1)
        sx1278.get_configuration()
        sx1278.set_mode_receive()

        while True:
            print("-" * 30)
            print(f"CURRENT RSSI: {sx1278.get_current_rssi()} dBm")
            # Принимаем данные
            data = sx1278.read_data()
            print(f"DATA: {data}")

            #formatted_string = ' '.join(f'{byte:02X}' for byte in data)
            #print(formatted_string) 
            # Преобразование байтовой строки в форматированную строку
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
            print(f"Message Counter: {sx1278.message_counter}")
            
    else:
        print("Not connected")

except KeyboardInterrupt:
    GPIO.cleanup()