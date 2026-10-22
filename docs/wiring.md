# ESP-FC wiring examples and PIN mapping

ESP32 MCUs allows to remap pins, so the wiring is not final and you can remap intputs and outputs to your needs. To change pin function go to the CLI and use `get pin` command to check current assignment. For example, to set first output to pin 1 use command

`set pin_output_0 1`

Tu unmap pin function use -1 as pin number

`set pin_output_3 -1`

> [!NOTE]
> There are still some limitations in remapping. Not all pins can be used with all functions. Some pins are input only, others are strapping pins and may prevent from booting if used incorrectly. please consult MCU documentation for available options. Default layout is proven to work.

## Default I2C pin mapping for gyro modules

| Module Pin | CLI Name         | ESP32 | ESP32-S3 |
|------------|------------------|------:|---------:|
| SCK/SCL    | `pin_i2c_scl`    | 22    | 10       |
| SDA/SDI    | `pin_i2c_sda`    | 21    | 9        |

> [!NOTE]
> On ESP32-C3 and RP2040 I2C driver accepts only pins from 1 to 31, on ESP8266 from 1 to 15

## Default SPI pin mapping gyro modules

| Module Pin  | CLI Name         | ESP32 | ESP32-S3 |
|-------------|------------------|------:|---------:|
| SCK/SCL     | `pin_spi_0_sck`  | 18    | 12       |
| SDA/SDI     | `pin_spi_0_mosi` | 23    | 11       |
| SAO/SDO/ADO | `pin_spi_0_miso` | 19    | 13       |
| NCS         | `pin_spi_cs_0`   |  5    |  8       |
| CSB*        | `pin_spi_cs_1`   | 13    |  7       |

**Note:** `CSB` is required for barometer on 10-DOF MPU-9250 modules

> [!TIP]
> For better performance preffer SPI

## Default Servo/Motor output mapping

| Motor  | CLI name       | ESP32 | ESP32-S3 |
|-------:|----------------|------:|---------:|
| 1      | `pin_output_0` | 27    | 39       |
| 2      | `pin_output_1` | 25    | 40       |
| 3      | `pin_output_2` | 4     | 41       |
| 4      | `pin_output_3` | 12    | 42       |

## Default Uart/Serial pin mapping

| Uart | CLI name          | ESP32 | ESP32-S3 |
|-----:|-------------------|------:|---------:|
| RX 1 | `pin_serial_0_rx` |  3    | 44       |
| TX 1 | `pin_serial_0_tx` |  1    | 43       |
| RX 2 | `pin_serial_1_rx` | 32    | 15       |
| TX 2 | `pin_serial_1_tx` | 33    | 16       |
| RX 3 | `pin_serial_2_rx` | 16    | 17       |
| TX 3 | `pin_serial_2_tx` | 17    | 18       |

## Default Analog pin mapping

| Uart    | CLI name          | ESP32 | ESP32-S3 |
|--------:|-------------------|------:|---------:|
| Voltage | `pin_input_adc_vbat` |  36   | 1        |
| Current | `pin_input_adc_ibat` |  39   | 4        |

> [!NOTE]
> On ESP32 choose only pins assigned to ADC1 channels

## Default PPM receiver pin mapping

| Uart    | CLI name       | ESP32 | ESP32-S3 |
|--------:|----------------|------:|---------:|
| PPM     | `pin_input_rx` |  35   | 6        |

## Other pin functions

| CLI name            | ESP32 | ESP32-S3 | Comment       |
|---------------------|------:|---------:|---------------|
| `pin_buzzer`        |  0    | 5        | Status buzzer |
| `pin_led`           |  26   | -        | Status led    |

## Deneyap Kart v2 pin mapping

Build with `pio run -e deneyap_kart_v2`. Board labels are given in brackets.

| Function       | CLI name             | GPIO | Board label |
|----------------|----------------------|-----:|-------------|
| I2C SDA (imu)  | `pin_i2c_sda`        | 47   | SDA         |
| I2C SCL (imu)  | `pin_i2c_scl`        | 21   | SCL         |
| Gyro SCK       | `pin_spi_0_sck`      | 41   | SCK         |
| Gyro MOSI      | `pin_spi_0_mosi`     | 39   | MO          |
| Gyro MISO      | `pin_spi_0_miso`     | 40   | MI          |
| Gyro CS        | `pin_spi_cs_0`       | 42   | SS          |
| Baro CS        | `pin_spi_cs_1`       | 38   | D8          |
| Motor 1        | `pin_output_0`       | 15   | D19         |
| Motor 2        | `pin_output_1`       | 16   | D18         |
| Motor 3        | `pin_output_2`       | 17   | D17         |
| Motor 4        | `pin_output_3`       | 18   | D16         |
| Receiver RX    | `pin_serial_0_rx`    | 44   | RX          |
| Receiver TX    | `pin_serial_0_tx`    | 43   | TX          |
| UART 2 RX      | `pin_serial_1_rx`    | 2    | D1          |
| UART 2 TX      | `pin_serial_1_tx`    | 1    | D0          |
| UART 3 RX      | `pin_serial_2_rx`    | 11   | D12         |
| UART 3 TX      | `pin_serial_2_tx`    | 10   | D13         |
| Voltage        | `pin_input_adc_vbat` | 4    | A0          |
| Current        | `pin_input_adc_ibat` | 5    | A1          |
| PPM            | `pin_input_rx`       | 7    | D20         |
| Buzzer         | `pin_buzzer`         | 48   | D9          |
| Status led     | `pin_led`            | 46   | onboard RGB |
| Button         | `pin_button`         | 0    | BOOT key    |

> [!NOTE]
> The board has onboard LSM6DSM imu on I2C bus (address 0x6A), it is detected as `LSM6DSO`.
> Optional external SPI gyro (ICM-42688-P, BMI160, MPU-6500, ...) on SPI header takes precedence over onboard imu.
> GPIO 12, 13, 14 are reserved for the onboard microSD slot.

## Example ESP32 SPI MPU-6500/MPU-9250 gyro

![ESP-FC ESP32 SPI Wiring](./images/esp-fc-esp32_spi_wiring.png)

## Example ESP32 I2C MPU-6050 gyro

![ESP-FC ESP32 I2C Wiring](./images/esp-fc-esp32_i2c_wiring.png)

## Example ESP8266 I2C MPU-6050 gyro

ESP8266 has limited ability to remap pins, use `get pin` command to list available options.

![ESP-FC ESP8266 I2C Wiring](./images/espfc_wemos_d1_mini_wiring.png)
