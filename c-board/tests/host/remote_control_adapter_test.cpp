#include <cstdint>
#include <cstdio>
#include <cstring>

#include "remote_control_adapter.hpp"

static unsigned failures;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,      \
                         #condition);                                          \
            failures++;                                                        \
        }                                                                      \
    } while (false)

class FakeBackend final {
public:
    struct Config {
        bool succeed;
    };

    bool init(const Config &config)
    {
        return config.succeed;
    }

    void read(RemoteControlState &state) const
    {
        state = next;
    }

    static RemoteControlState next;
};

RemoteControlState FakeBackend::next;

static RemoteControlState ValidState(void)
{
    RemoteControlState state{};
    state.type = REMOTE_CONTROL_TYPE_ET08;
    state.link_online = 1U;
    state.data_valid = 1U;
    state.switch_valid_mask = 1UL;
    state.switches[0] = REMOTE_SWITCH_DOWN;
    state.buttons_supported = REMOTE_BUTTON_TRIGGER;
    return state;
}

static void TestEdgesAndReconnectBaseline(void)
{
    rm::remote::RemoteControlAdapter<FakeBackend> adapter;
    RemoteControlState output{};

    CHECK(adapter.init({true}));
    CHECK(adapter.initialized());

    FakeBackend::next = ValidState();
    FakeBackend::next.keys_down = 1UL << 2U;
    CHECK(adapter.read(10U, output));
    CHECK(output.sample_sequence == 1U);
    CHECK(output.sample_time_ms == 10U);
    CHECK(output.keys_pressed == 0U);
    CHECK(output.keys_released == 0U);
    CHECK(output.switch_changed_mask == 0U);

    FakeBackend::next.switches[0] = REMOTE_SWITCH_UP;
    FakeBackend::next.keys_down = 1UL << 5U;
    FakeBackend::next.buttons_down = REMOTE_BUTTON_TRIGGER;
    CHECK(adapter.read(20U, output));
    CHECK(output.sample_sequence == 2U);
    CHECK(output.switch_changed_mask == 1UL);
    CHECK(output.keys_pressed == (1UL << 5U));
    CHECK(output.keys_released == (1UL << 2U));
    CHECK(output.buttons_pressed == REMOTE_BUTTON_TRIGGER);
    CHECK(output.buttons_released == 0U);

    FakeBackend::next.data_valid = 0U;
    FakeBackend::next.link_online = 0U;
    CHECK(!adapter.read(30U, output));
    CHECK(output.sample_sequence == 3U);

    /* 重连首帧只建立基线，不把断线期间的位置变化伪造成按键沿。 */
    FakeBackend::next = ValidState();
    FakeBackend::next.switches[0] = REMOTE_SWITCH_UP;
    FakeBackend::next.buttons_down = REMOTE_BUTTON_TRIGGER;
    CHECK(adapter.read(40U, output));
    CHECK(output.sample_sequence == 4U);
    CHECK(output.switch_changed_mask == 0U);
    CHECK(output.buttons_pressed == 0U);
    CHECK(output.buttons_released == 0U);
}

static void TestFailedInitializationStaysSafe(void)
{
    rm::remote::RemoteControlAdapter<FakeBackend> adapter;
    RemoteControlState output;

    std::memset(&output, 0xA5, sizeof(output));
    CHECK(!adapter.init({false}));
    CHECK(!adapter.initialized());
    CHECK(!adapter.read(100U, output));
    CHECK(output.type == REMOTE_CONTROL_TYPE_NONE);
    CHECK(output.link_online == 0U);
    CHECK(output.data_valid == 0U);
    CHECK(output.sample_sequence == 0U);
}

int main()
{
    TestEdgesAndReconnectBaseline();
    TestFailedInitializationStaysSafe();

    if (failures != 0U) {
        std::fprintf(stderr, "%u remote-adapter checks failed\n", failures);
        return 1;
    }
    std::puts("remote-adapter checks passed");
    return 0;
}
