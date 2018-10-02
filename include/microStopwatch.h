#ifndef MICROSTOPWATCH_H_
#define MICROSTOPWATCH_H_

#include "../include/function.h"
#include <boost/algorithm/string.hpp>

namespace myClass
{
    class MicroStopwatch
    {
        public:
            int64_t elapsed;
            std::string name;
            boost::posix_time::ptime tictic;
            boost::posix_time::ptime toctoc;
            MicroStopwatch()
            {
                elapsed = 0;
            }
            MicroStopwatch(std::string name)
            {
                elapsed = 0;
                this->name = name;
            }
            void tic()
            {
                tictic = boost::posix_time::microsec_clock::local_time ();
            }
            int64_t toc()
            {
                toctoc = boost::posix_time::microsec_clock::local_time ();
                this->elapsed += (toctoc - tictic).total_microseconds();
                return (toctoc - tictic).total_microseconds();
            }
            std::string toc_string()
            {
                toctoc = boost::posix_time::microsec_clock::local_time ();
                return myFunction::commaFix((toctoc - tictic).total_microseconds());
            }
            void toc_print_string()
            {
                std::cerr << this->name << ": "<< this->toc_string() << " us" <<  std::endl;
            }
            std::string elapsed_string()
            {
                return myFunction::commaFix(elapsed);
            }
            void elapsed_print_string()
            {
                std::cerr << this->name << ": "<< this->elapsed_string() << " us" <<  std::endl;
            }
            int64_t toc_pre()
            {
                return (toctoc - tictic).total_microseconds();
            }
            void clear()
            {
                this->elapsed = 0;
            }
    };
}
#endif