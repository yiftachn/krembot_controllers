

#ifndef PORTING_EXAMPLE_MYCODE_H
#define PORTING_EXAMPLE_MYCODE_H

#include "Krembot/krembot.h"

class MyCode {
private:
    Krembot & _krembot;
    SandTimer timer;
    int _action {0};
public:

    MyCode(Krembot & krembot);
    void doSomething();
};


#endif //PORTING_EXAMPLE_MYCODE_H
