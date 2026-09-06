#ifndef SBV_LOG_H
#define SBV_LOG_H

void sbv_log_printf(const char *format, ...);

#define LOG_INFO(fmt, ...)  sbv_log_printf("[INFO]  " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  sbv_log_printf("[WARN]  " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) sbv_log_printf("[ERROR] " fmt "\n", ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) sbv_log_printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)

#endif /* SBV_LOG_H */