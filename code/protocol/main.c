#include "protocol.h"

#include <stdio.h>

int main(int argc, const char *argv[]) {

    protocol_data data_0 = {
        .type = 2,
        .id = 2,
        .value = 100
    };

    protocol_frame frame = protocol_encode(data_0);

    protocol_data data_1 = protocol_decode(frame);

    printf("%d %d %d \n", data_1.type, data_1.id, data_1.value);
    printf("%d \n", protocol_checksum_check(frame));

    frame.a = 45;
    frame.b = 46;

    printf("%d \n", protocol_checksum_check(frame));


    /*
    SENDING EXAMPLE
    protocol_data data = {
        .type = 1,
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
