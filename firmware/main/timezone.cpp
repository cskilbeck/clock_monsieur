#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>

// Define maximum size for the TZ string
#define TZ_MAX_LEN 128

/**
 * @brief Helper function to convert seconds offset to TZ format string (e.g., -28800 -> "8").
 * The output offset is UTC - Local Time, meaning the raw offset sign is negated.
 * @param offset_sec The raw offset from UTC in seconds (e.g., -28800 for PST).
 * @param buffer Output buffer for the formatted string.
 * @param max_len Max size of the buffer.
 */
static void format_offset(long offset_sec, char *buffer, size_t max_len)
{
    // POSIX TZ offset is LOCAL time offset WEST of UTC (negated raw offset).
    long total_sec = -offset_sec;    // Apply the sign flip for POSIX format

    char sign = (total_sec < 0) ? '-' : '+';

    // Ensure total_sec is positive for calculations
    if(total_sec < 0) {
        total_sec = -total_sec;
    }

    long hours = total_sec / 3600;
    long minutes = (total_sec % 3600) / 60;
    long seconds = total_sec % 60;

    // Standard TZ format is HH[:MM[:SS]].
    // Only include sign if the offset is negative (i.e., local time is East of UTC).
    if(minutes == 0 && seconds == 0) {
        if(sign == '-') {
            snprintf(buffer, max_len, "%c%ld", sign, hours);
        } else {
            snprintf(buffer, max_len, "%ld", hours);
        }
    } else if(seconds == 0) {
        snprintf(buffer, max_len, "%c%ld:%02ld", sign, hours, minutes);
    } else {
        snprintf(buffer, max_len, "%c%ld:%02ld:%02ld", sign, hours, minutes, seconds);
    }
}

/**
 * @brief Converts a DST transition epoch time into the POSIX rule format: M<month>.<week>.<day>/<time>
 * @param epoch_time The time_t value of the DST transition (in local time *before* the change).
 * @param buffer Output buffer to store the formatted rule string.
 * @param max_len Max size of the buffer.
 * @return True on success, False on error.
 */
static bool format_posix_rule(time_t epoch_time, char *buffer, size_t max_len)
{
    struct tm tm_info;

    // Use localtime_r to fill tm_info with the local time components
    // Note: The TZ environment variable must be set *prior* to calling this
    // for this function to work correctly in a fresh environment.
    // In the context of ESP-IDF, setting TZ *before* calling this is usually necessary
    // if the system's current rules are not already correct.
    if(localtime_r(&epoch_time, &tm_info) == NULL) {
        return false;
    }

    int m = tm_info.tm_mon + 1;    // Month (1-12)
    int d = tm_info.tm_wday;       // Day of week (0=Sunday, 6=Saturday)
    int day_of_month = tm_info.tm_mday;

    // 1. Determine 'n' (the week number: 1st, 2nd, 3rd, 4th, or 5th/Last)
    int n;
    int occurrence = (day_of_month - 1) / 7 + 1;

    // Check if the current occurrence is the last one in the month
    // We calculate the last day of the month for comparison
    struct tm last_day_check = tm_info;
    last_day_check.tm_mday = 1;    // Start at the 1st of the month

    // Advance to the next month
    int next_month = last_day_check.tm_mon + 1;
    if(next_month > 11) {
        next_month = 0;
        last_day_check.tm_year++;
    }
    last_day_check.tm_mon = next_month;

    // Calculate the epoch for the first day of the next month
    time_t next_month_start = mktime(&last_day_check);

    // Calculate the epoch for the last day of the current month
    time_t current_month_end_epoch = next_month_start - 24 * 3600;

    // Get the structure for the last day
    if(localtime_r(&current_month_end_epoch, &last_day_check) == NULL) {
        return false;
    }

    int last_day_of_month = last_day_check.tm_mday;
    int last_occurrence = (last_day_of_month - 1) / 7 + 1;

    if(occurrence == last_occurrence) {
        n = 5;    // It is the last occurrence of that day in the month
    } else {
        n = occurrence;
    }

    // 2. Format the time part (HH:MM:SS)
    char time_str[9];    // HH:MM:SS
    // The format specifiers must be correct based on the time components derived from the epoch
    strftime(time_str, sizeof(time_str), "%H:%M:%S", &tm_info);

    // 3. Assemble the rule: M<m>.<n>.<d>/<time>
    snprintf(buffer, max_len, "M%d.%d.%d/%s", m, n, d, time_str);

    return true;
}


/**
 * @brief Creates the full proleptic POSIX TZ string for ESP-IDF/newlib.
 * @param std_abbr Standard Time Abbreviation (e.g., "GMT").
 * @param std_offset_sec Raw Standard Offset from UTC in seconds (e.g., 0).
 * @param dst_abbr DST Abbreviation (e.g., "BST").
 * @param dst_offset_sec Raw DST Offset from UTC in seconds (e.g., 3600).
 * @param dst_start_epoch Epoch time when DST BEGINS (e.g., M3.5.0/1:00:00).
 * @param dst_end_epoch Epoch time when DST ENDS (e.g., M10.5.0/2:00:00).
 * @return A dynamically allocated string containing the TZ setting, or NULL on failure.
 * The caller is responsible for freeing the returned string.
 */
char *create_posix_tz_string(const char *std_abbr, long std_offset_sec, const char *dst_abbr, long dst_offset_sec, time_t dst_start_epoch,
                             time_t dst_end_epoch)
{
    char *tz_string = (char *)malloc(TZ_MAX_LEN);
    if(tz_string == NULL) {
        return NULL;    // Allocation failure
    }

    char std_offset_buffer[16];
    char dst_offset_buffer[16];
    char start_rule_buffer[64];
    char end_rule_buffer[64];

    // 1. Format Standard Time Offset
    format_offset(std_offset_sec, std_offset_buffer, sizeof(std_offset_buffer));

    // Check if DST is applicable (both start and end rules provided)
    if(dst_start_epoch > 0 && dst_end_epoch > 0 && dst_abbr != NULL && strlen(dst_abbr) > 0) {

        // 2. Format DST Start and End Rules
        // Note: The POSIX format requires the start rule first, then the end rule.
        if(!format_posix_rule(dst_start_epoch, start_rule_buffer, sizeof(start_rule_buffer)) ||
           !format_posix_rule(dst_end_epoch, end_rule_buffer, sizeof(end_rule_buffer))) {
            free(tz_string);
            return NULL;    // Rule formatting failed
        }

        // 3. Format DST Offset (if needed)
        format_offset(dst_offset_sec, dst_offset_buffer, sizeof(dst_offset_buffer));

        // The DST offset is OPTIONAL if it's exactly 1 hour different from the standard offset (3600 seconds).
        // Since StdOffset is UTC - Local, the DST offset must be 3600 seconds different on the raw offset.
        bool is_default_dst_offset = (dst_offset_sec == (std_offset_sec + 3600));

        // 4. Assemble the full TZ string with DST
        // Format: StdAbbr StdOffset DstAbbr [DstOffset],StartRule,EndRule
        snprintf(tz_string, TZ_MAX_LEN, "%s%s%s%s,%s,%s", std_abbr, std_offset_buffer, dst_abbr,
                 is_default_dst_offset ? "" : dst_offset_buffer,    // Omit if default 1 hour difference
                 start_rule_buffer, end_rule_buffer);
    } else {
        // 4. Assemble the Standard Time only string (no DST rule)
        // Format: StdAbbr StdOffset
        snprintf(tz_string, TZ_MAX_LEN, "%s%s", std_abbr, std_offset_buffer);
    }

    return tz_string;
}