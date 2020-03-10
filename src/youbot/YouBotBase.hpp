/**
 * THIS IS THE MOCK FILE
 **/
#include <iostream>

#define YOUBOT_CONFIGURATIONS_DIR "./"
#define radian 57.2957795131

#define LOG(tag) std::cout << (tag) << ": "
#define info "INFO"
#define warning "WARNING"

namespace youbot {
    class YouBotBase
    {
    private:
        /* data */
    public:
        YouBotBase(const char* mockName, const char* mockFileName) { /* mock */ }
        ~YouBotBase() { /* mock */ }
        void doJointCommutation() { /* mock */ }
    };
    
}