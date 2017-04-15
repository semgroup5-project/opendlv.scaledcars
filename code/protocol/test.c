#include "protocol.h"

#include <stdio.h>

int main(int argc, const char *argv[])
{

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

            count++;
        }
    }

    printf("count=%d total=%d error=%f \n", count, total, total / count);

    protocol_frame _frame;
    _frame.a = 45;
    _frame.b = 46;

    printf("0==%d \n", protocol_checksum_check(_frame));

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
