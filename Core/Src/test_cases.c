#include "test_cases.h"
#include "sx1278.h"

#define NUM_TEST_CASES 5
DataFrame testCases[NUM_TEST_CASES] = {
    {
        .dev_id = 0x02,
        .timestamp = 1718016660,
        .latitude = 21.123100,
        .longitude = 105.654100,
        .checksum = 0
    },
    {
        .dev_id = 0x02,
        .timestamp = 1718016720,
        .latitude = 21.125000,
        .longitude = 105.654500,
        .checksum = 0
    },
    {
        .dev_id = 0x02,
        .timestamp = 1718016780,
        .latitude = 21.123900,
        .longitude = 105.654900,
        .checksum = 0
    },
    {
        .dev_id = 0x02,
        .timestamp = 1718016840,
        .latitude = 21.123500,
        .longitude = 105.653500,
        .checksum = 0
    },
    {
        .dev_id = 0x02,
        .timestamp = 1718016900,
        .latitude = 21.123500,
        .longitude = 105.654500,
        .checksum = 0
    }
};

void initializeTestCases(void) {
    for (int i = 0; i < NUM_TEST_CASES; i++) {
        testCases[i].checksum = 0;
        uint8_t* dataPtr = (uint8_t*)&testCases[i];
        for (int j = 0; j < 13; j++) {
            testCases[i].checksum ^= dataPtr[j];
        }
    }
}

void sendTestCase(int index) {
    if (index >= 0 && index < NUM_TEST_CASES) {
        sx1278_send(&hspi1, (uint8_t*)&testCases[index], sizeof(DataFrame));
    }
}
