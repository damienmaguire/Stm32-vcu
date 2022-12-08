#ifndef TEST_H_INCLUDED
#define TEST_H_INCLUDED
#include <iostream>

class IUnitTest
{
    public:
        virtual ~IUnitTest() {}
        virtual void RunTest() = 0;
};

extern int _failedAssertions;

#define STRING(s) #s
#define ASSERT(c) \
   if (c)   \
      cout << "Test " << __FILE__ << "::" << __func__ << " passed." << endl; \
   else \
   {  \
      cout << "Assertion failed: " << STRING(c) << " in " __FILE__ " : " << __LINE__ << endl;    \
      _failedAssertions++; \
   }

#endif // TEST_H_INCLUDED
