#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "minmea.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE (2048)


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
    char *nmea_sentence = (char *)malloc(BUF_SIZE/2);
    if (!nmea_sentence)
    {
        ESP_LOGE("GPS", "Failed to allocate memory for NMEA sentence");
    }
    int nmea_index = 0;

    while (1)
    {
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, (size_t *)&length));
        length = uart_read_bytes(UART_NUM, data, length, 100 / portTICK_PERIOD_MS);
        
        if (length > 0)
        {
            // print the data from uart
            // ESP_LOGI("GPS", "Received data: %.*s\n\n\n", length, data);

            for (int i = 0; i < length; i++)
            {
                // Append character to NMEA sentence buffer
                nmea_sentence[nmea_index++] = data[i];

                // Check if the end of the sentence is reached
                if (data[i] == '\n')
                {
                    nmea_sentence[nmea_index] = '\0'; // Null-terminate the sentence

                    if (strncmp(nmea_sentence, "$GPRMC", 6) == 0)
                    {
                        struct minmea_sentence_rmc rmc;
                        if (minmea_parse_rmc(&rmc, nmea_sentence))
                        {
                            if (rmc.valid)
                            {
                                ESP_LOGI("GPS", "$GPRMC Latitude: %f \nLongitude: %f \nSpeed: %f \n Date: %d-%d-%d\n Time: %d:%d:%d\n",
                                        minmea_tocoord(&rmc.latitude), minmea_tocoord(&rmc.longitude), minmea_tofloat(&rmc.speed), rmc.date.day, rmc.date.month, rmc.date.year, rmc.time.hours, rmc.time.minutes, rmc.time.seconds);
                            }
                            else
                            {
                                ESP_LOGE("GPS", "Invalid RMC");
                            }
                        }
                    }
                    else if (strncmp(nmea_sentence, "$GPGGA", 6) == 0)
                    {
                        struct minmea_sentence_gga gga;
                        if (minmea_parse_gga(&gga, nmea_sentence))
                        {
                            if (gga.fix_quality > 0)
                            {
                                ESP_LOGI("GPS", "$GPGGA Latitude: %f \nLongitude: %f \nAltitude: %f \nFix quality: %d \nTime: %d:%d:%d \nSatellites Tracked: %d \nhdop: %f \nHeight of geoid: %f \n",
                                        minmea_tocoord(&gga.latitude), minmea_tocoord(&gga.longitude), minmea_tofloat(&gga.altitude), gga.fix_quality, gga.time.hours, gga.time.minutes, gga.time.seconds, gga.satellites_tracked, minmea_tofloat(&gga.hdop), minmea_tofloat(&gga.height));
                            }
                            else
                            {
                                ESP_LOGE("GPS", "Invalid GGA");
                            }
                        }
                    }
                    else if (strncmp(nmea_sentence, "$GPVTG", 6) == 0)
                    {
                        struct minmea_sentence_vtg vtg;
                        if (minmea_parse_vtg(&vtg, nmea_sentence))
                        {
                            if (vtg.faa_mode != MINMEA_FAA_MODE_NOT_VALID)
                            {
                                // HEADING ISN'T WORKING BUT I THINK IT SHOULD, TO BE TESTED
                                ESP_LOGI("GPS", "$GPVTG True Track: %f \nSpeed: %f \n",
                                        minmea_tofloat(&vtg.true_track_degrees), minmea_tofloat(&vtg.speed_kph));
                            } else {
                                ESP_LOGE("GPS", "Invalid VTG");
                            }
                        }
                    }
                    else if (strncmp(nmea_sentence, "$GPGSA", 6) == 0)
                    {
                        struct minmea_sentence_gsa gsa;
                        if (minmea_parse_gsa(&gsa, nmea_sentence))
                        {
                            /*
                            if (gsa.fix_type > 0)
                            {
                                // Display the mode (Manual or Automatic)
                                ESP_LOGI("GPS", "$GPGSA Mode: %c \nFix type: %d \nPDOP: %.2f \nHDOP: %.2f \nVDOP: %.2f\n",
                                         gsa.mode, gsa.fix_type, minmea_tofloat(&gsa.pdop), minmea_tofloat(&gsa.hdop), minmea_tofloat(&gsa.vdop));

                                // Display the IDs of satellites used for the fix
                                ESP_LOGI("GPS", "Satellites used for fix:");
                                for (int i = 0; i < 12; i++)
                                {
                                    if (gsa.sats[i] != 0)
                                    { // Satellite IDs are non-zero
                                        ESP_LOGI("GPS", "Satellite ID: %d", gsa.sats[i]);
                                    }
                                }
                            }
                            else
                            {
                                ESP_LOGE("GPS", "Invalid GSA");
                            }
                            */
                        }
                    }
                    else if (strncmp(nmea_sentence, "$GPGLL", 6) == 0)
                    {
                        struct minmea_sentence_gll gll;
                        if (minmea_parse_gll(&gll, nmea_sentence))
                        {
                            /*
                            if (gll.status == 'A')
                            {
                                ESP_LOGI("GPS", "$GPGLL Latitude: %f \nLongitude: %f \nTime: %d:%d:%d \nStatus: %c \nMode: %c\n",
                                        minmea_tocoord(&gll.latitude), minmea_tocoord(&gll.longitude), gll.time.hours, gll.time.minutes, gll.time.seconds, gll.status, gll.mode);
                            }
                            else
                            {
                                ESP_LOGE("GPS", "Invalid GLL");
                            }
                            */
                        }
                    }
                    else if (strncmp(nmea_sentence, "$GPGSV", 6) == 0)
                    {
                        struct minmea_sentence_gsv gsv;
                        if (minmea_parse_gsv(&gsv, nmea_sentence))
                        {
                            /*
                            ESP_LOGI("GPS", "$GSV: message %d of %d", gsv.msg_nr, gsv.total_msgs);
                            ESP_LOGI("GPS", "$GSV: satellites in view: %d", gsv.total_sats);
                            for (int i = 0; i < 4; i++)
                                ESP_LOGI("GPS", "$GSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm",
                                        gsv.sats[i].nr,
                                        gsv.sats[i].elevation,
                                        gsv.sats[i].azimuth,
                                        gsv.sats[i].snr);
                            */
                        }
                    }

                    // Reset the index for the next sentence
                    nmea_index = 0;
                }
            }
    }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    free(nmea_sentence);
}
