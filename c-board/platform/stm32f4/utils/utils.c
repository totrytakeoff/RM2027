#include "utils.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    char *buffer;
    size_t capacity;
    size_t total;
} FormatWriter;

typedef enum {
    FORMAT_LENGTH_DEFAULT = 0,
    FORMAT_LENGTH_HH,
    FORMAT_LENGTH_H,
    FORMAT_LENGTH_L,
    FORMAT_LENGTH_LL,
    FORMAT_LENGTH_Z,
} FormatLength;

typedef struct {
    bool left;
    bool plus;
    bool space;
    bool zero;
    bool alternate;
} FormatFlags;

static void WriterPut(FormatWriter *writer, char value)
{
    if ((writer->capacity != 0U) &&
        (writer->total < (writer->capacity - 1U))) {
        writer->buffer[writer->total] = value;
    }
    writer->total++;
}

static void WriterRepeat(FormatWriter *writer, char value, size_t count)
{
    for (size_t i = 0U; i < count; ++i) {
        WriterPut(writer, value);
    }
}

static void WriterWrite(FormatWriter *writer, const char *text, size_t length)
{
    for (size_t i = 0U; i < length; ++i) {
        WriterPut(writer, text[i]);
    }
}

static void WriterFinish(FormatWriter *writer)
{
    if (writer->capacity == 0U) {
        return;
    }

    if (writer->total < writer->capacity) {
        writer->buffer[writer->total] = '\0';
    } else {
        writer->buffer[writer->capacity - 1U] = '\0';
    }
}

static size_t UnsignedDigits(uint64_t value,
                             unsigned base,
                             bool uppercase,
                             char *reversed)
{
    const char *digits = uppercase
                             ? "0123456789ABCDEF"
                             : "0123456789abcdef";
    size_t count = 0U;

    do {
        reversed[count++] = digits[value % base];
        value /= base;
    } while (value != 0U);

    return count;
}

static void WriteInteger(FormatWriter *writer,
                         uint64_t magnitude,
                         bool negative,
                         unsigned base,
                         bool uppercase,
                         FormatFlags flags,
                         int width,
                         int precision,
                         bool force_prefix)
{
    char reversed[32];
    char prefix[3];
    size_t digit_count;
    size_t prefix_length = 0U;
    size_t precision_zeros = 0U;
    size_t padding = 0U;

    digit_count = UnsignedDigits(magnitude, base, uppercase, reversed);
    if ((precision == 0) && (magnitude == 0U)) {
        digit_count = 0U;
    }

    if (negative) {
        prefix[prefix_length++] = '-';
    } else if (flags.plus) {
        prefix[prefix_length++] = '+';
    } else if (flags.space) {
        prefix[prefix_length++] = ' ';
    }

    if ((flags.alternate || force_prefix) && (base == 16U) &&
        ((magnitude != 0U) || force_prefix)) {
        prefix[prefix_length++] = '0';
        prefix[prefix_length++] = uppercase ? 'X' : 'x';
    } else if (flags.alternate && (base == 8U) &&
               ((digit_count == 0U) ||
                (reversed[digit_count - 1U] != '0'))) {
        prefix[prefix_length++] = '0';
    }

    if ((precision > 0) && ((size_t)precision > digit_count)) {
        precision_zeros = (size_t)precision - digit_count;
    }
    if ((width > 0) &&
        ((size_t)width > prefix_length + precision_zeros + digit_count)) {
        padding = (size_t)width - prefix_length - precision_zeros - digit_count;
    }

    if (!flags.left && !(flags.zero && (precision < 0))) {
        WriterRepeat(writer, ' ', padding);
    }
    WriterWrite(writer, prefix, prefix_length);
    if (!flags.left && flags.zero && (precision < 0)) {
        WriterRepeat(writer, '0', padding);
    }
    WriterRepeat(writer, '0', precision_zeros);
    while (digit_count != 0U) {
        WriterPut(writer, reversed[--digit_count]);
    }
    if (flags.left) {
        WriterRepeat(writer, ' ', padding);
    }
}

static void WriteText(FormatWriter *writer,
                      const char *text,
                      size_t length,
                      FormatFlags flags,
                      int width)
{
    size_t padding = 0U;

    if ((width > 0) && ((size_t)width > length)) {
        padding = (size_t)width - length;
    }
    if (!flags.left) {
        WriterRepeat(writer, ' ', padding);
    }
    WriterWrite(writer, text, length);
    if (flags.left) {
        WriterRepeat(writer, ' ', padding);
    }
}

static void WriteFloat(FormatWriter *writer,
                       double value,
                       FormatFlags flags,
                       int width,
                       int precision)
{
    char text[96];
    char reversed[32];
    size_t length = 0U;
    size_t integer_digits;
    size_t padding = 0U;
    uint64_t scale = 1U;
    uint64_t scaled;
    uint64_t integer_part;
    uint64_t fractional_part;
    bool negative = signbit(value) != 0;

    if (precision < 0) {
        precision = 6;
    }
    if (precision > 9) {
        precision = 9;
    }

    if (negative) {
        text[length++] = '-';
        value = -value;
    } else if (flags.plus) {
        text[length++] = '+';
    } else if (flags.space) {
        text[length++] = ' ';
    }

    if (isnan(value)) {
        memcpy(&text[length], "nan", 3U);
        length += 3U;
    } else if (isinf(value)) {
        memcpy(&text[length], "inf", 3U);
        length += 3U;
    } else {
        for (int i = 0; i < precision; ++i) {
            scale *= 10U;
        }
        if (value > ((double)UINT64_MAX / (double)scale)) {
            memcpy(&text[length], "overflow", 8U);
            length += 8U;
        } else {
            scaled = (uint64_t)((value * (double)scale) + 0.5);
            integer_part = scaled / scale;
            fractional_part = scaled % scale;
            integer_digits = UnsignedDigits(integer_part,
                                             10U,
                                             false,
                                             reversed);
            while (integer_digits != 0U) {
                text[length++] = reversed[--integer_digits];
            }

            if ((precision != 0) || flags.alternate) {
                text[length++] = '.';
            }
            if (precision != 0) {
                uint64_t divisor = scale / 10U;
                for (int i = 0; i < precision; ++i) {
                    text[length++] =
                        (char)('0' + ((fractional_part / divisor) % 10U));
                    divisor /= 10U;
                }
            }
        }
    }

    if ((width > 0) && ((size_t)width > length)) {
        padding = (size_t)width - length;
    }
    if (!flags.left && !flags.zero) {
        WriterRepeat(writer, ' ', padding);
    }
    if (!flags.left && flags.zero && (length != 0U) &&
        ((text[0] == '-') || (text[0] == '+') || (text[0] == ' '))) {
        WriterPut(writer, text[0]);
        WriterRepeat(writer, '0', padding);
        WriterWrite(writer, &text[1], length - 1U);
    } else {
        if (!flags.left && flags.zero) {
            WriterRepeat(writer, '0', padding);
        }
        WriterWrite(writer, text, length);
    }
    if (flags.left) {
        WriterRepeat(writer, ' ', padding);
    }
}

static int ParseNumber(const char **cursor)
{
    int value = 0;

    while ((**cursor >= '0') && (**cursor <= '9')) {
        if (value < 10000) {
            value = (value * 10) + (**cursor - '0');
        }
        (*cursor)++;
    }
    return value;
}

static int WriterResult(const FormatWriter *writer)
{
    return (writer->total > (size_t)INT_MAX) ? INT_MAX : (int)writer->total;
}

int RmFormat_Vsnprintf(char *buffer,
                       size_t buffer_size,
                       const char *format,
                       va_list args)
{
    FormatWriter writer = {
        .buffer = buffer,
        .capacity = buffer_size,
        .total = 0U,
    };
    const char *cursor = format;

    if ((format == NULL) || ((buffer == NULL) && (buffer_size != 0U))) {
        return -1;
    }

    while (*cursor != '\0') {
        FormatFlags flags = {0};
        FormatLength length = FORMAT_LENGTH_DEFAULT;
        int width = 0;
        int precision = -1;
        char specifier;

        if (*cursor != '%') {
            WriterPut(&writer, *cursor++);
            continue;
        }
        cursor++;

        for (;;) {
            if (*cursor == '-') flags.left = true;
            else if (*cursor == '+') flags.plus = true;
            else if (*cursor == ' ') flags.space = true;
            else if (*cursor == '0') flags.zero = true;
            else if (*cursor == '#') flags.alternate = true;
            else break;
            cursor++;
        }

        if (*cursor == '*') {
            width = va_arg(args, int);
            cursor++;
            if (width < 0) {
                flags.left = true;
                width = -width;
            }
        } else {
            width = ParseNumber(&cursor);
        }

        if (*cursor == '.') {
            cursor++;
            if (*cursor == '*') {
                precision = va_arg(args, int);
                cursor++;
                if (precision < 0) precision = -1;
            } else {
                precision = ParseNumber(&cursor);
            }
        }

        if ((*cursor == 'h') && (cursor[1] == 'h')) {
            length = FORMAT_LENGTH_HH;
            cursor += 2;
        } else if (*cursor == 'h') {
            length = FORMAT_LENGTH_H;
            cursor++;
        } else if ((*cursor == 'l') && (cursor[1] == 'l')) {
            length = FORMAT_LENGTH_LL;
            cursor += 2;
        } else if (*cursor == 'l') {
            length = FORMAT_LENGTH_L;
            cursor++;
        } else if (*cursor == 'z') {
            length = FORMAT_LENGTH_Z;
            cursor++;
        }

        specifier = *cursor;
        if (specifier == '\0') {
            WriterPut(&writer, '%');
            break;
        }
        cursor++;

        if ((specifier == 'd') || (specifier == 'i')) {
            int64_t value;
            if (length == FORMAT_LENGTH_LL) value = va_arg(args, long long);
            else if (length == FORMAT_LENGTH_L) value = va_arg(args, long);
            else if (length == FORMAT_LENGTH_Z) value = va_arg(args, ptrdiff_t);
            else {
                int raw = va_arg(args, int);
                if (length == FORMAT_LENGTH_HH) value = (signed char)raw;
                else if (length == FORMAT_LENGTH_H) value = (short)raw;
                else value = raw;
            }
            WriteInteger(&writer,
                         (value < 0) ? (UINT64_C(0) - (uint64_t)value)
                                     : (uint64_t)value,
                         value < 0,
                         10U,
                         false,
                         flags,
                         width,
                         precision,
                         false);
        } else if ((specifier == 'u') || (specifier == 'o') ||
                   (specifier == 'x') || (specifier == 'X')) {
            uint64_t value;
            unsigned base = (specifier == 'o') ? 8U
                             : ((specifier == 'x') || (specifier == 'X'))
                                   ? 16U
                                   : 10U;
            if (length == FORMAT_LENGTH_LL) value = va_arg(args, unsigned long long);
            else if (length == FORMAT_LENGTH_L) value = va_arg(args, unsigned long);
            else if (length == FORMAT_LENGTH_Z) value = va_arg(args, size_t);
            else {
                unsigned raw = va_arg(args, unsigned);
                if (length == FORMAT_LENGTH_HH) value = (unsigned char)raw;
                else if (length == FORMAT_LENGTH_H) value = (unsigned short)raw;
                else value = raw;
            }
            flags.plus = false;
            flags.space = false;
            WriteInteger(&writer,
                         value,
                         false,
                         base,
                         specifier == 'X',
                         flags,
                         width,
                         precision,
                         false);
        } else if (specifier == 'p') {
            flags.plus = false;
            flags.space = false;
            WriteInteger(&writer,
                         (uintptr_t)va_arg(args, void *),
                         false,
                         16U,
                         false,
                         flags,
                         width,
                         precision,
                         true);
        } else if (specifier == 'c') {
            const char value = (char)va_arg(args, int);
            WriteText(&writer, &value, 1U, flags, width);
        } else if (specifier == 's') {
            const char *value = va_arg(args, const char *);
            size_t text_length;
            if (value == NULL) value = "(null)";
            text_length = strlen(value);
            if ((precision >= 0) && ((size_t)precision < text_length)) {
                text_length = (size_t)precision;
            }
            WriteText(&writer, value, text_length, flags, width);
        } else if ((specifier == 'f') || (specifier == 'F') ||
                   (specifier == 'e') || (specifier == 'E') ||
                   (specifier == 'g') || (specifier == 'G')) {
            WriteFloat(&writer,
                       va_arg(args, double),
                       flags,
                       width,
                       precision);
        } else if (specifier == '%') {
            WriterPut(&writer, '%');
        } else {
            WriterPut(&writer, '%');
            WriterPut(&writer, specifier);
        }
    }

    WriterFinish(&writer);
    return WriterResult(&writer);
}

int RmFormat_Snprintf(char *buffer,
                      size_t buffer_size,
                      const char *format,
                      ...)
{
    int result;
    va_list args;

    va_start(args, format);
    result = RmFormat_Vsnprintf(buffer, buffer_size, format, args);
    va_end(args);
    return result;
}

int safe_vsnprintf(char *buffer,
                   size_t buffer_size,
                   const char *format,
                   va_list args)
{
    return RmFormat_Vsnprintf(buffer, buffer_size, format, args);
}

int safe_snprintf(char *buffer,
                  size_t buffer_size,
                  const char *format,
                  ...)
{
    int result;
    va_list args;

    va_start(args, format);
    result = RmFormat_Vsnprintf(buffer, buffer_size, format, args);
    va_end(args);
    return result;
}
