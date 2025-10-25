#ifndef MATHFUNC_H
#define MATHFUNC_H
#endif // !MATHFUNC_H

// 取最值
#define PEAK(x, max)           \
    do                         \
    {                          \
        if ((x) > (max))       \
            (x) = (max);       \
                               \
        else if ((x) < -(max)) \
            (x) = -(max);      \
    } while (0)

#define MIN(x, min)      \
    do                   \
    {                    \
        if ((x) > (min)) \
            (x) = (min); \
    } while (0)

#define MAX(x, max)      \
    do                   \
    {                    \
        if ((x) < (max)) \
            (x) = (max); \
    } while (0)
// 取绝对值

static inline int signum(int x)
{
    return (x > 0) - (x < 0);
}

#define ABS(x) ((x) > 0 ? (x) : -(x)) //???
