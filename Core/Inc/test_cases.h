#ifndef TEST_CASES_H
#define TEST_CASES_H

#include "gps.h"

#define NUM_TEST_CASES 5

extern DataFrame testCases[NUM_TEST_CASES];
void initializeTestCases(void);
void sendTestCase(int index);

#endif /* TEST_CASES_H */
