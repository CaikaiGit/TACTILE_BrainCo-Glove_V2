#ifndef APPLICATION_VERSION_H
#define APPLICATION_VERSION_H

// 版本号定义
#define APPLICATION_VERSION       "QNST"
#define APPLICATION_VERSION_MAJOR 0
#define APPLICATION_VERSION_MINOR 1
#define APPLICATION_VERSION_PATCH 0910

// 辅助宏 - 修正版本
#define STRINGIFY(x) STRINGIFY_(x)
#define STRINGIFY_(x) #x
#define CONCAT(a, b) CONCAT_(a, b)
#define CONCAT_(a, b) a##b

#define APPLICATION_VERSION_FULL  APPLICATION_VERSION "." \
                                  STRINGIFY(APPLICATION_VERSION_MAJOR) "." \
                                  STRINGIFY(APPLICATION_VERSION_MINOR) "." \
                                  STRINGIFY(APPLICATION_VERSION_PATCH)

#endif // APPLICATION_VERSION_H
