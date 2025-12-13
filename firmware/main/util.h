#pragma once

#include <cstring>
#include <cstdint>
#include <cstddef>

#include "esp_log.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////////////////

#define ASSERT(x)                                                                                                            \
    do {                                                                                                                     \
        if(!(x)) {                                                                                                           \
            printf("\033[0;31m\n\n=====\n\nASSERT '%s' failed\nLINE %d\nFILE %s\n\n=====\n\033[0m", #x, __LINE__, __FILE__); \
            fflush(stdout);                                                                                                  \
            fsync(fileno(stdout));                                                                                           \
            abort();                                                                                                         \
        }                                                                                                                    \
    } while(false)

//////////////////////////////////////////////////////////////////////

void delay_ms(int ms);
void delay_secs(int seconds);

//////////////////////////////////////////////////////////////////////

// #define NO_LOGGING

#if defined(NO_LOGGING)

#define LOG_NOP \
    do {        \
    } while(0)

#define LOG_CONTEXT(x)

#define LOG_ERROR(...) LOG_NOP
#define LOG_WARN(...) LOG_NOP
#define LOG_INFO(...) LOG_NOP
#define LOG_VERBOSE(...) LOG_NOP
#define LOG_DEBUG(...) LOG_NOP

#define LOG_SET_LEVEL(...) LOG_NOP

#define LOG_BUFFER(...) LOG_NOP

#define LOG_FLUSH(...) LOG_NOP

#else

#define LOG_CONTEXT(x) static char const *__log_tag [[maybe_unused]] = x

#define LOG_ERROR(...) ESP_LOGE(__log_tag, __VA_ARGS__)
#define LOG_WARN(...) ESP_LOGW(__log_tag, __VA_ARGS__)
#define LOG_INFO(...) ESP_LOGI(__log_tag, __VA_ARGS__)
#define LOG_VERBOSE(...) ESP_LOGV(__log_tag, __VA_ARGS__)
#define LOG_DEBUG(...) ESP_LOGD(__log_tag, __VA_ARGS__)

#define LOG_SET_LEVEL(channel, level) esp_log_level_set(channel, level)

#define LOG_BUFFER(level, data, len)                 \
    do {                                             \
        if(LOG_LOCAL_LEVEL >= level)                 \
            log_buffer(__log_tag, data, len, level); \
    } while(0)

#define LOG_FLUSH(...)         \
    do {                       \
        fflush(stdout);        \
        fsync(fileno(stdout)); \
    } while(0)

#endif

//////////////////////////////////////////////////////////////////////

#define ESP_CHECK(x)                                                                          \
    do {                                                                                      \
        esp_err_t __err = (x);                                                                \
        if(__err != ESP_OK) {                                                                 \
            LOG_ERROR("%s failed: 0x%08x (%s)", #x, (uint32_t)__err, esp_err_to_name(__err)); \
            return __err;                                                                     \
        }                                                                                     \
    } while(0)

//////////////////////////////////////////////////////////////////////

#define ESP_LOG_ERR(x)                                                                        \
    do {                                                                                      \
        esp_err_t __err = (x);                                                                \
        if(__err != ESP_OK) {                                                                 \
            LOG_ERROR("%s failed: 0x%08x (%s)", #x, (uint32_t)__err, esp_err_to_name(__err)); \
        }                                                                                     \
    } while(0)

//////////////////////////////////////////////////////////////////////

#define ESP_VOID(x)                                                                           \
    do {                                                                                      \
        esp_err_t __err = (x);                                                                \
        if(__err != ESP_OK) {                                                                 \
            LOG_ERROR("%s failed: 0x%08x (%s)", #x, (uint32_t)__err, esp_err_to_name(__err)); \
            return;                                                                           \
        }                                                                                     \
    } while(0)

//////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

//////////////////////////////////////////////////////////////////////
// templates not allowed in extern "C" section

#ifdef __cplusplus

#include <utility>

namespace
{
    //////////////////////////////////////////////////////////////////////

    template <typename T, size_t S> struct simplestack_t
    {
        T buffer[S];
        size_t size = 0;

        void clear()
        {
            size = 0;
        }

        bool empty() const
        {
            return size == 0;
        }

        void push(T v)
        {
            if(size < S) {
                buffer[size] = v;
                size += 1;
            }
        }

        T pop()
        {
            if(size != 0) {
                size -= 1;
                return buffer[size];
            }
            return (T)0;
        }
    };

    //////////////////////////////////////////////////////////////////////

    inline uint8_t int_to_bcd(int n)
    {
        return (n % 10) | ((n / 10) << 4);
    }

    //////////////////////////////////////////////////////////////////////

    inline int bcd_to_int(uint8_t n)
    {
        return (n & 0xf) + ((n >> 4) * 10);
    }

    //////////////////////////////////////////////////////////////////////

    template <typename T, size_t N> constexpr size_t countof(T const (&)[N]) noexcept
    {
        return N;
    }

    //////////////////////////////////////////////////////////////////////

    template <typename T> T min(T x, T y)
    {
        return (x <= y) ? x : y;
    }

    //////////////////////////////////////////////////////////////////////

    template <typename T> T max(T x, T y)
    {
        return (x >= y) ? x : y;
    }

    //////////////////////////////////////////////////////////////////////

    template <typename F> class defer_finalizer
    {
        F f;
        bool moved;

    public:
        template <typename T> defer_finalizer(T &&f_) : f(std::forward<T>(f_)), moved(false)
        {
        }

        defer_finalizer(const defer_finalizer &) = delete;

        defer_finalizer(defer_finalizer &&other) : f(std::move(other.f)), moved(other.moved)
        {
            other.moved = true;
        }

        void cancel()
        {
            moved = true;
        }

        ~defer_finalizer()
        {
            if(!moved) {
                f();
            }
        }
    };

    //////////////////////////////////////////////////////////////////////

    static struct
    {
        template <typename F> defer_finalizer<F> operator<<(F &&f)
        {
            return defer_finalizer<F>(std::forward<F>(f));
        }
    } deferrer __attribute__((__used__));

}    // namespace

//////////////////////////////////////////////////////////////////////

#define DEFER_TOKENPASTE(x, y) x##y
#define DEFER_TOKENPASTE2(x, y) DEFER_TOKENPASTE(x, y)
#define SCOPED auto DEFER_TOKENPASTE2(__deferred_lambda_call, __COUNTER__) = deferrer <<
#define DEFERRED deferrer <<
#define DEFER(X) \
    SCOPED[=]    \
    {            \
        X;       \
    };

template <typename T> [[maybe_unused]] constexpr char const *enum_to_string(T value)
{
    return nullptr;
}

//////////////////////////////////////////////////////////////////////

template <std::size_t N> struct string_literal
{
    char value[N];
    constexpr string_literal(const char (&str)[N])
    {
        for(int i = 0; i < N; ++i)
            value[i] = str[i];
    }
    constexpr operator char const *() const
    {
        return value;
    }
};

//////////////////////////////////////////////////////////////////////

template <typename T> constexpr size_t count_enum_values()
{
    size_t count = 0;
    while(enum_to_string((T)count) != nullptr) {
        count += 1;
    }
    return count;
}
//////////////////////////////////////////////////////////////////////

template <typename T> constexpr size_t enum_names_length()
{
    size_t length = 0;
    constexpr size_t count = count_enum_values<T>();

    for(size_t i = 0; i < count; ++i) {
        const char *name = enum_to_string(static_cast<T>(i));
        for(const char *p = name; *p; ++p) {
            ++length;
        }
        if(i + 1 < count) {
            ++length;    // For the '|' separator
        }
    }

    return length;
}

//////////////////////////////////////////////////////////////////////

template <typename... Ts> constexpr size_t total_enum_names_length()
{
    size_t total = 0;
    size_t index = 0;

    ((total += enum_names_length<Ts>() + (index++ > 0 ? 1 : 0)), ...);

    return total;
}

//////////////////////////////////////////////////////////////////////

template <typename... Ts> constexpr auto enum_names()
{
    constexpr size_t total_length = total_enum_names_length<Ts...>();

    char buffer[total_length + 1] = {};
    size_t pos = 0;
    size_t type_index = 0;

    auto append_enum = [&]<typename T>() {
        // Add space separator between enum types (but not before first)
        if(type_index++ > 0) {
            buffer[pos++] = ' ';
        }

        constexpr size_t count = count_enum_values<T>();
        for(size_t i = 0; i < count; ++i) {
            const char *name = enum_to_string(static_cast<T>(i));

            // Copy the name
            for(const char *p = name; *p; ++p) {
                buffer[pos++] = *p;
            }

            // Add separator if not last value
            if(i + 1 < count) {
                buffer[pos++] = '|';
            }
        }
    };

    (append_enum.template operator()<Ts>(), ...);

    buffer[pos] = '\0';

    return string_literal(buffer);
}

//////////////////////////////////////////////////////////////////////

#define ENUM(enum_name, underlying_type, ...)                                          \
    enum class enum_name : underlying_type                                             \
    {                                                                                  \
        __VA_ARGS__                                                                    \
    };                                                                                 \
                                                                                       \
    template <> [[maybe_unused]] constexpr char const *enum_to_string(enum_name value) \
    {                                                                                  \
        switch(value) {                                                                \
            ENUM_CASES(enum_name, __VA_ARGS__)                                         \
        default:                                                                       \
            return nullptr;                                                            \
        }                                                                              \
        return nullptr;                                                                \
    }

// Helper to convert comma-separated names into case statements
// This uses a trick with MAP macro
#define ENUM_CASE(enum_name, name) \
    case enum_name::name:          \
        return #name;

// Macro magic to apply ENUM_CASE to each argument
#define PARENS ()
#define EXPAND(...) EXPAND4(EXPAND4(EXPAND4(EXPAND4(__VA_ARGS__))))
#define EXPAND4(...) EXPAND3(EXPAND3(EXPAND3(EXPAND3(__VA_ARGS__))))
#define EXPAND3(...) EXPAND2(EXPAND2(EXPAND2(EXPAND2(__VA_ARGS__))))
#define EXPAND2(...) EXPAND1(EXPAND1(EXPAND1(EXPAND1(__VA_ARGS__))))
#define EXPAND1(...) __VA_ARGS__

#define FOR_EACH(macro, enum_name, ...) __VA_OPT__(EXPAND(FOR_EACH_HELPER(macro, enum_name, __VA_ARGS__)))
#define FOR_EACH_HELPER(macro, enum_name, a1, ...) macro(enum_name, a1) __VA_OPT__(FOR_EACH_AGAIN PARENS(macro, enum_name, __VA_ARGS__))
#define FOR_EACH_AGAIN() FOR_EACH_HELPER

#define ENUM_CASES(enum_name, ...) FOR_EACH(ENUM_CASE, enum_name, __VA_ARGS__)

#endif
