#ifndef _LOG_UTILS_H_
#define _LOG_UTILS_H_

#include <iostream>
#include <sstream>
// Macros and other utilties

#define PRINT(x) std::cout << x << std::endl;
#define DEBUG(x) std::cout << "\033[36m" << x << "\033[0m" << std::endl;
#define ERROR(x) std::cout << "\033[31m" << x << "\033[0m" << std::endl;
#define SUCCESS(x) std::cout << "\033[32m" << x << "\033[0m" << std::endl;
#define WARN(x) std::cout << "\033[33m" << x << "\033[0m" << std::endl;


#endif // _LOG_UTILS_H_
