# CR95HF
Mbed library for CR95HF (X-Nucleo NFC03A1).
**Only ISO/IEC 14443A is suported at the moment.**

It has been tested with:
- MIFARE Classic 1K (4-byte UID)
- MIFARE Ultralight (7-byte UID)

## Example
```cpp
#include "mbed.h"
#include "CR95HF.h"

CR95HF nfc(D8, D2);

int main() {
    if (nfc.init() != CR95HF_ERROR_OK) {
        return;
    }

    if (nfc.setProtocol(CR95HF::ISO14443A) != CR95HF_ERROR_OK) {
        return;
    }

    while (true) {
        ThisThread::sleep_for(500ms);

        if (!nfc.isTagInRange()) {
            continue;
        }

        uint8_t *uid = new uint8_t[10];
        uint8_t card_type = 0;

        uint8_t tag_len = nfc.getTagUID(uid);

        if (tag_len > 0) {
            printf("UID: %s\n", tr_array(uid, tag_len));
        }

        delete[] uid;
    }
}
```

## Debug example
`mbed_app.json`
```json
{
  "config": {
    "trace-level": {
      "help": "Options are TRACE_LEVEL_ERROR,TRACE_LEVEL_WARN,TRACE_LEVEL_INFO,TRACE_LEVEL_DEBUG",
      "macro_name": "MBED_TRACE_MAX_LEVEL",
      "value": "TRACE_LEVEL_DEBUG"
    }
  },
  "target_overrides": {
    "*": {
      "mbed-trace.enable": true,
      "target.printf_lib": "std"
    }
  }
}

```

```cpp
#include "mbed.h"
#include "CR95HF.h"

#if MBED_CONF_MBED_TRACE_ENABLE
#include "mbed-trace/mbed_trace.h"
static Mutex trace_mutex;

static void trace_wait() {
    trace_mutex.lock();
}

static void trace_release() {
    trace_mutex.unlock();
}

void trace_init() {
    mbed_trace_init();
    mbed_trace_mutex_wait_function_set(trace_wait);
    mbed_trace_mutex_release_function_set(trace_release);
}
#endif

CR95HF nfc(D8, D2);

int main() {
#if MBED_CONF_MBED_TRACE_ENABLE
    trace_init();
#endif

    if (nfc.init() != CR95HF_ERROR_OK) {
        return;
    }

    if (nfc.setProtocol(CR95HF::ISO14443A) != CR95HF_ERROR_OK) {
        return;
    }

    while (true) {
        ThisThread::sleep_for(500ms);

        if (!nfc.isTagInRange()) {
            continue;
        }

        uint8_t *uid = new uint8_t[10];
        uint8_t card_type = 0;

        uint8_t tag_len = nfc.getTagUID(uid, &card_type);

        if (tag_len > 0) {
            printf("UID: %s\n", tr_array(uid, tag_len));

            if ((card_type & 0b00100100) == 0) {
                printf("TAG is MIFARE\n");

            } else if ((card_type & 0b00100100) == 0b00100000) {
                printf("TAG is ISO14443-4\n");
            }
        }

        delete[] uid;
    }
}
```
