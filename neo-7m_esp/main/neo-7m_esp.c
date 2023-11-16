#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "minmea.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE (1024)


void app_main()
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    // Buffer to store a complete NMEA sentence
    char nmea_sentence[BUF_SIZE];
    int nmea_index = 0;

    while (1)
    {
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, (size_t *)&length));
        length = uart_read_bytes(UART_NUM, data, length, 100 / portTICK_PERIOD_MS);
        
        if (length > 0)
        {
            // print the data from uart
            ESP_LOGI("GPS", "Received data: %.*s\n\n\n", length, data);

            /*
            for (int i = 0; i < length; i++)
            {
                // Append character to NMEA sentence buffer
                nmea_sentence[nmea_index++] = data[i];

                // Check if the end of the sentence is reached
                if (data[i] == '\n')
                {
                    nmea_sentence[nmea_index] = '\0'; // Null-terminate the sentence
                    // ESP_LOGI("GPS", "Complete NMEA Sentence: %s", nmea_sentence);

                    if (strncmp(nmea_sentence, "$GPGGA", 6) == 0)
                    {
                        struct minmea_sentence_gga gga;
                        if (minmea_parse_gga(&gga, nmea_sentence))
                        {
                            if (gga.fix_quality > 0) // Check if the fix quality is good
                            {
                                ESP_LOGI("GPS", "Valid GGA: latitude=%.8f, longitude=%.8f",
                                         minmea_tocoord(&gga.latitude),
                                         minmea_tocoord(&gga.longitude));
                                // Display the time
                                ESP_LOGI("GPS", "UTC Time: %02d:%02d:%02d",
                                         gga.time.hours,
                                         gga.time.minutes,
                                         gga.time.seconds);
                            }
                            else
                            {
                                ESP_LOGI("GPS", "Invalid GGA");
                            }
                        }
                    }
                    else if (strncmp(nmea_sentence, "$GPVTG", 6) == 0)
                    {
                        struct minmea_sentence_vtg vtg;
                        if (minmea_parse_vtg(&vtg, nmea_sentence))
                        {
                            ESP_LOGI("GPS", "Speed over ground: %.2f knots, %.2f km/h",
                                     minmea_tofloat(&vtg.speed_knots),
                                     minmea_tofloat(&vtg.speed_kph));
                        }
                    }
                    else if (strncmp(nmea_sentence, "$GPGSA", 6) == 0)
                    {
                        struct minmea_sentence_gsa gsa;
                        if (minmea_parse_gsa(&gsa, nmea_sentence))
                        {
                            ESP_LOGI("GPS", "Fix type: %d, PDOP: %.2f, HDOP: %.2f, VDOP: %.2f",
                                     gsa.fix_type,
                                     minmea_tofloat(&gsa.pdop),
                                     minmea_tofloat(&gsa.hdop),
                                     minmea_tofloat(&gsa.vdop));
                        }
                    }
                    // else if (strncmp(nmea_sentence, "$GPGSV", 6) == 0)
                    // {
                    //     struct minmea_sentence_gsv gsv;
                    //     if (minmea_parse_gsv(&gsv, nmea_sentence))
                    //     {
                    //         ESP_LOGI("GPS", "Satellite %d: PRN %d, Elevation %d, Azimuth %d, SNR %d",
                    //                  i + 1, gsv.sats[i].nr, gsv.sats[i].elevation, gsv.sats[i].azimuth, gsv.sats[i].snr);
                    //     }
                    // }

                    // Reset the index for the next sentence
                    nmea_index = 0;
                }
            } */
        }
        vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
}
