//
// Created by Jeremy Cote on 2023-08-26.
//

#include <unity.h>
#include <Arduino.h>

int fibonacci(int n) {
    if (n <= 1) {
        return n;
    }
    return fibonacci(n - 1) + fibonacci(n - 2);
}

void test_fibonacci(void) {
    TEST_ASSERT_EQUAL(0, fibonacci(0));
    TEST_ASSERT_EQUAL(1, fibonacci(1));
    TEST_ASSERT_EQUAL(1, fibonacci(2));
    TEST_ASSERT_EQUAL(2, fibonacci(3));
    TEST_ASSERT_EQUAL(3, fibonacci(4));
    TEST_ASSERT_EQUAL(5, fibonacci(5));
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_fibonacci);
    UNITY_END();
}

void loop() {}
