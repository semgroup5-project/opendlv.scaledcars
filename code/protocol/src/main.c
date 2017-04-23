#include "protocol.h"
#include "serial.h"
#include "arduino.h"

#include <stdio.h>
#include <unistd.h>

void __on_read(uint8_t b)
{
    printf(">> read '%c' \n", b);
}

void __on_write(uint8_t b)
{
    printf("<< write '%c' \n", b);
}

int main(int argc, const char *argv[])
{
    /*
     * Protocol
     */

    int count = 0;
    int total = 0;

    for (int id = 0; id < 8; id++) {
        for (int value = 0; value < 256; value++) {
            total++;

            protocol_data _data = {
                .id = id,
                .value = value
            };

            protocol_frame frame = protocol_encode(_data);
            protocol_data data = protocol_decode(frame);

            if (!(data.id == _data.id)) continue;
            if (!(data.value == _data.value)) continue;

            if (!(protocol_checksum_check(frame))) continue;

            if (!(protocol_get_byte_index(frame.a) == 0)) continue;
            if (!(protocol_get_byte_index(frame.b) == 1)) continue;

            count++;
        }
    }

    float ratio = count / total;

    printf("count=%d total=%d ratio=%f \n", count, total, ratio);

    protocol_frame _frame;
    _frame.a = 45;
    _frame.b = 46;

    printf("0==%d \n", protocol_checksum_check(_frame));

    protocol_data _data1 = {
        .id = 7,
        .value = 255
    };

    protocol_data _data2 = {
        .id = 0,
        .value = 0
    };

    protocol_frame _frame1 = protocol_encode(_data1);
    protocol_frame _frame2 = protocol_encode(_data2);

    protocol_state protocol;
    protocol.frame.a = 0;
    protocol.frame.b = 0;
    protocol_receive(&protocol, _frame1.a);
    protocol_receive(&protocol, _frame1.b);

    printf("%d==%d \n", true, protocol.valid);
    printf("%d==%d \n", _data1.id, protocol.data.id);
    printf("%d==%d \n", _data1.value, protocol.data.value);

    /*
     * Serial
     */

    serial_state *serial = serial_new();

    serial->on_write = &__on_write;
    serial->on_read = &__on_read;

    serial_open(serial, "/dev/ttyACM0", 115200);
    serial_handshake(serial, '\n');
    serial_start(serial);

    serial_send(serial, _data1);
    serial_send(serial, _data2);

    sleep(20);

    protocol_data echo;
    while (!serial_receive(serial, &echo));
    printf("%d==%d %d==%d \n", _data1.id, echo.id, _data1.value, echo.value);
    while (!serial_receive(serial, &echo));
    printf("%d==%d %d==%d \n", _data2.id, echo.id, _data2.value, echo.value);

    sleep(10);

    serial_stop(serial);
    serial_free(serial);

    return 0;

    /*
    SENDING EXAMPLE

    protocol_data data = {
        .id = 1,
        .value = 1
    };

    protocol_frame = protocol_encode(data);

    Serial.write(protocol_frame.a);
    Serial.write(protocol_frame.b);
     */


    /*
    RECEIVING EXAMPLE

    protocol_state state;

    while (true)
    {
        if (Serial.available()) {
            protocol_receive(&state, Serial.read());
            if (state.valid) {
                // do something with state->data
            }
        }
    }
    */

    return 0;
}
