#ifndef APPLICATION_UTILS_H
#define APPLICATION_UTILS_H
//
//#define MAX(a, b)                                                                                          \
//    ({                                                                                                     \
//        __typeof__(a) _a = (a);                                                                            \
//        __typeof__(b) _b = (b);                                                                            \
//        _a > _b ? _a : _b;                                                                                 \
//    })
//
//#define MIN(a, b)                                                                                          \
//    ({                                                                                                     \
//        __typeof__(a) _a = (a);                                                                            \
//        __typeof__(b) _b = (b);                                                                            \
//        _a < _b ? _a : _b;                                                                                 \
//    })

//#define CLAMP(a, b, c) \
//    ({ __typeof__(a) _a = (a); \
//       __typeof__(b) _b = (b); \
//       __typeof__(c) _c = (c); \
//       (_a < _b) ? _b : ((_a > _c) ? _c : _a); })

static inline int32_t MAX(int32_t a, int32_t b) {
    return (a > b) ? a : b;
}

static inline int32_t MIN(int32_t a, int32_t b) {
    return (a < b) ? a : b;
}

static inline int CLAMP(int value, int min_value, int max_value) {
    return (value < min_value) ? min_value : ((value > max_value) ? max_value : value);
}
#endif // APPLICATION_UTILS_H
