#ifndef DB_R70_V2_LOG_H
#define DB_R70_V2_LOG_H

#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_DEBUG 4
#define LOG_LEVEL_TRACE 5

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_NONE
#endif

#define LOG_ENABLED(level) (LOG_LEVEL >= (level))

#define LOG_BLOCK(level, code) \
  do {                         \
    if (LOG_ENABLED(level)) {  \
      code;                    \
    }                          \
  } while (0)

#define LOG_PRINTLN(level, stream, text) \
  do {                                   \
    if (LOG_ENABLED(level)) {            \
      (stream).println(text);            \
    }                                    \
  } while (0)

#endif  // DB_R70_V2_LOG_H
