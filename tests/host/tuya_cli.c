// Try tr_tuya against a real device from a computer on the same network.
//
//   cc -std=c11 -D_DEFAULT_SOURCE -D_DARWIN_C_SOURCE -Icomponents/tr_tuya/include \
//      tests/host/tuya_cli.c components/tr_tuya/tr_tuya.c \
//      tests/host/tuya_crypto_host.c -o .tools/tuya_cli
//   TUYA_KEY=<32 hex> .tools/tuya_cli <ip> <id> <33|34>            status
//   TUYA_KEY=<32 hex> .tools/tuya_cli <ip> <id> <33|34> <dp> on|off switch
//
// The key comes from the environment, so it is not in the shell history.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tr_tuya.h"

int main(int argc, char **argv)
{
    tr_tuya_dev_t dev = {0};
    if ((argc != 4 && argc != 6) || !tr_tuya_key_from_hex(getenv("TUYA_KEY"), dev.key)) {
        fprintf(stderr, "usage: TUYA_KEY=<32 hex> %s <ip> <id> <33|34> [<dp> on|off]\n", argv[0]);
        return 2;
    }
    dev.ip = argv[1];
    dev.id = argv[2];
    dev.version = atoi(argv[3]);
    if (argc == 6) {
        const int r = tr_tuya_set(&dev, atoi(argv[4]), strcmp(argv[5], "on") == 0);
        printf("set dp %s %s: %s\n", argv[4], argv[5], tr_tuya_err_text(r));
        if (r != 0) {
            return 1;
        }
    }
    tr_tuya_dp_t dps[32];
    const int n = tr_tuya_status(&dev, dps, 32);
    if (n < 0) {
        printf("status: %s\n", tr_tuya_err_text(n));
        return 1;
    }
    for (int i = 0; i < n; i++) {
        printf("%s%d=%d", i ? " " : "", dps[i].dp, dps[i].value);
    }
    printf("\n");
    return 0;
}
