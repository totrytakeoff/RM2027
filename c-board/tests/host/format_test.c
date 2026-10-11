#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "utils.h"

static unsigned failures;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,           \
                    #condition);                                               \
            failures++;                                                        \
        }                                                                      \
    } while (false)

static void CheckFormat(const char *expected, const char *format, ...)
{
    char buffer[128];
    int result;
    va_list args;

    va_start(args, format);
    result = RmFormat_Vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    CHECK(result == (int)strlen(expected));
    CHECK(strcmp(buffer, expected) == 0);
}

static void TestIntegerFormatting(void)
{
    CheckFormat("x=-42 u=17 hex=0x002a",
                "x=%d u=%u hex=%#06x",
                -42,
                17U,
                42U);
    CheckFormat("00001234", "%08lx", 0x1234UL);
    CheckFormat("-9223372036854775808", "%lld", (-9223372036854775807LL - 1LL));
    CheckFormat("[abc  ]", "[%-5.3s]", "abcdef");
}

static void TestFloatFormatting(void)
{
    CheckFormat("2.00", "%.2f", 1.999);
    CheckFormat("-001.50", "%+07.2f", -1.5);
    CheckFormat("+3.", "%+#.0f", 3.0);
}

static void TestBoundsAndCompatibilityAlias(void)
{
    char buffer[5];
    int result = RmFormat_Snprintf(buffer, sizeof(buffer), "abcdef");

    CHECK(result == 6);
    CHECK(strcmp(buffer, "abcd") == 0);
    CHECK(RmFormat_Snprintf(NULL, 0U, "value=%u", 12U) == 8);

    result = safe_snprintf(buffer, sizeof(buffer), "%u", 123U);
    CHECK(result == 3);
    CHECK(strcmp(buffer, "123") == 0);
}

int main(void)
{
    TestIntegerFormatting();
    TestFloatFormatting();
    TestBoundsAndCompatibilityAlias();

    if (failures != 0U) {
        fprintf(stderr, "%u formatter checks failed\n", failures);
        return 1;
    }

    puts("formatter checks passed");
    return 0;
}
