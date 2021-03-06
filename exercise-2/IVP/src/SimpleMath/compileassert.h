#ifndef _COMPILE_ASSERT_H
#define _COMPILE_ASSERT_H

/* 
 * This is a simple compile time assertion tool taken from:
 *   http://blogs.msdn.com/b/abhinaba/archive/2008/10/27/c-c-compile-time-asserts.aspx
 * written by Abhinaba Basu!
 *
 * Thanks!
 */

#ifdef __cplusplus

#define JOIN( X, Y ) JOIN2(X,Y)
#define JOIN2( X, Y ) X##Y

namespace my_static_assert
{
    template <bool> struct STATIC_ASSERT_FAILURE;
    template <> struct STATIC_ASSERT_FAILURE<true> { enum { value = 1 }; };

    template<int x> struct my_static_assert_test{};
}

#define COMPILE_ASSERT(x) \
    typedef ::my_static_assert::my_static_assert_test<\
        sizeof(::my_static_assert::STATIC_ASSERT_FAILURE< (bool)( x ) >)>\
            JOIN(_my_static_assert_typedef, __LINE__)

#else // __cplusplus

#define COMPILE_ASSERT(x) extern int __dummy[(int)x]

#endif // __cplusplus

#define VERIFY_EXPLICIT_CAST(from, to) COMPILE_ASSERT(sizeof(from) == sizeof(to)) 

// _COMPILE_ASSERT_H_
#endif
