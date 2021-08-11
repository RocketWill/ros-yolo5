#include <time.h>
#include <exception>

// simple nested stack-based inline high-precision timer class
// link with -lrt
// throws exceptions when used improperly

#define MAX_CLOCKS 16

class tictoc {
    private:
        class clock_overflow_exception : public std::exception
        {
            virtual const char* what() const throw() {
                return "Exceeded MAX_CLOCKS!\n";
            }
        } tictoc_maxclock_ex;

        class clock_underflow_exception : public std::exception
        {
            virtual const char* what() const throw() {
                return "No clocks active\n";
            }
        } tictoc_underflow_ex;

        struct timespec clocks[MAX_CLOCKS];
        int depth;
    public:
        tictoc() : depth(0)
        {}

        inline void tic(void) {
            if(depth == MAX_CLOCKS)
                throw(tictoc_maxclock_ex);

            clock_gettime(CLOCK_MONOTONIC_RAW, &clocks[depth++]);
        }

        inline double toc(void) {
            struct timespec final;
            clock_gettime(CLOCK_MONOTONIC_RAW, &final);

            if(!depth)
                throw(tictoc_underflow_ex);

            depth--;
            double ret = 1e-9*(final.tv_nsec-clocks[depth].tv_nsec)
                        + (final.tv_sec-clocks[depth].tv_sec);
            return ret;
        }
};