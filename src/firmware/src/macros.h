#ifndef MACROS_H
#define MACROS_H

#include <Esp.h>

#define LED_BUILTIN 2

#define RCCHECK(fn)                                         \
    {                                                       \
        rcl_ret_t temp_rc = fn;                             \
        if ((temp_rc != RCL_RET_OK))                        \
        {                                                   \
            Serial.println("Critical failure in microROS"); \
            error_loop();                                   \
        }                                                   \
    }

#define RCSOFTCHECK(fn)                                          \
    {                                                            \
        rcl_ret_t temp_rc = fn;                                  \
        if ((temp_rc != RCL_RET_OK))                             \
        {                                                        \
            Serial.println("Failed Result: " + String(temp_rc)); \
        }                                                        \
    }


#ifdef DEBUG
#define PRINT_DBG(msg) Serial.printf("[DBG]" msg "\n");
#define PRINT_DBG_ARGS(msg, arg) Serial.printf("[DBG]" msg "\n", arg);
#else
#define PRINT_DBG(msg)
#define PRINT_DBG_ARGS(msg, arg)
#endif

// @brief Blink the LED and restart the ESP32 after 10 blinks
void error_loop()
{

#ifdef ESP32
    uint8_t i;

    while (i < 10)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);

        i++;
    }

    ESP.restart();
#else
    while (true)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
#endif
}

#endif