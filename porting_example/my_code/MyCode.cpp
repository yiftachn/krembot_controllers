

#include "MyCode.h"

MyCode::MyCode(Krembot & krembot) : _krembot{krembot} {

}

void MyCode::doSomething() {

    timer.start(1500);
    if (timer.finished()) {
        switch (_action++) {
            case 0: {
                _krembot.Base.drive(0, -50);
                break;
            }
            case 1: {
                _krembot.Led.write(0, 255, 0);
                break;
            }
            case 2: {
                _krembot.Base.drive(50, 0);
                break;
            }
            case 3: {
                _krembot.Led.write(255, 0, 0);
                break;
            }
        }
        if (_action > 3) _action = 0;
    }
}