//
// Created by Jeremy Cote on 2023-08-31.
//

#include <gtest/gtest.h>
#include "ObservedObject.hpp"
#include "DataQueue.hpp"
#include "ObservedDataQueue.hpp"

struct MockStruct {
    uint8_t a;
    uint8_t b;
};

TEST(ObservableObjectTests, ConstructWithStruct) {
    MockStruct m;
    m.a = 10;
    m.b = 20;

    ObservedDataQueue<MockStruct> data(10);
    data.add(m);

    EXPECT_EQ(data.getLatestValue().a, 10);
    EXPECT_EQ(data.getLatestValue().b, 20);

    m.a = 30;

    EXPECT_EQ(data.getLatestValue().a, 10);
    EXPECT_EQ(data.getLatestValue().b, 20);

    data.add(m);

    EXPECT_EQ(data.getLatestValue().a, 30);
    EXPECT_EQ(data.getLatestValue().b, 20);
}