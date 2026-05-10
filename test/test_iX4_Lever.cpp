/*
 * Drop-in unit test for Stm32-vcu's existing test/ harness (`make Test`).
 * Drop this file into test/, append an iX4LeverTest declaration to
 * test_list.h, and add iX4_Lever.o + test_iX4_Lever.o to OBJS in
 * test/Makefile (see proposals/firmware/bmw_g26_lever/INTEGRATION.md).
 *
 * Verifies the empirically-determined CRC8 parameters and gear decoding
 * against 12 fixtures sampled from the public openinverter forum captures
 * (thread #7028, 2022 BMW i4 G26).
 */

#include "iX4_Lever.h"
#include "test_fixtures.h"
#include "test_list.h"

#include <cstring>
#include <iostream>

using namespace std;

static void TestStaticCrcMatchesEveryFixture() {
    for (size_t i = 0; i < G26_LEVER_FIXTURE_COUNT; ++i) {
        uint8_t got = iX4_Lever::ComputeCrc8(g26_lever_fixtures[i].data);
        ASSERT(got == g26_lever_fixtures[i].expected_crc);
    }
}

static void TestMutatedCrcIsRejected() {
    uint8_t mutated[8];
    memcpy(mutated, g26_lever_fixtures[0].data, 8);
    mutated[6] ^= 0x01;
    ASSERT(!iX4_Lever::ValidateFrame(mutated));
}

static void TestEachGearByteDecodesCorrectly() {
    struct { uint8_t b6; Shifter::Sgear expect; } cases[] = {
        { 0x33, Shifter::PARK    },
        { 0x31, Shifter::NEUTRAL },
        { 0x32, Shifter::REVERSE },
        { 0x35, Shifter::DRIVE   },
    };
    for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) {
        for (size_t j = 0; j < G26_LEVER_FIXTURE_COUNT; ++j) {
            if (g26_lever_fixtures[j].expected_gear_byte != cases[i].b6) continue;
            iX4_Lever lever;
            uint32_t buf[2];
            memcpy(buf, g26_lever_fixtures[j].data, 8);
            lever.DecodeCAN(IX4_LEVER_CAN_ID, buf);
            Shifter::Sgear got;
            ASSERT(lever.GetGear(got));
            ASSERT(got == cases[i].expect);
            break;
        }
    }
}

static void TestBadCrcDoesNotChangeGear() {
    iX4_Lever lever;
    uint32_t buf[2];

    memcpy(buf, g26_lever_fixtures[0].data, 8);
    lever.DecodeCAN(IX4_LEVER_CAN_ID, buf);
    Shifter::Sgear before;
    lever.GetGear(before);
    ASSERT(before == Shifter::PARK);

    uint8_t corrupt[8];
    memcpy(corrupt, g26_lever_fixtures[2].data, 8); /* a Drive frame */
    corrupt[0] ^= 0xFF;
    memcpy(buf, corrupt, 8);
    lever.DecodeCAN(IX4_LEVER_CAN_ID, buf);

    Shifter::Sgear after;
    lever.GetGear(after);
    ASSERT(after == Shifter::PARK);
    ASSERT(lever.GetCrcFailCount() == 1);
}

static void TestBadCounterHighNibbleRejected() {
    iX4_Lever lever;
    uint8_t bad[8];
    memcpy(bad, g26_lever_fixtures[0].data, 8);
    bad[1] = 0xA5;
    bad[0] = iX4_Lever::ComputeCrc8(bad);
    uint32_t buf[2];
    memcpy(buf, bad, 8);
    lever.DecodeCAN(IX4_LEVER_CAN_ID, buf);
    ASSERT(lever.GetCounterFailCount() == 1);
}

void iX4LeverTest::RunTest() {
    TestStaticCrcMatchesEveryFixture();
    TestMutatedCrcIsRejected();
    TestEachGearByteDecodesCorrectly();
    TestBadCrcDoesNotChangeGear();
    TestBadCounterHighNibbleRejected();
}
