#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "globals.h"

namespace controller
{
    /**
     * handles commands 
     * 
     * @param raw_input     raw input string
     * */
    bool commandHandler(String raw_input);

    /**
     * handles debug commands
     * 
     * @param raw_input     raw input string
     * */
    void debugCommands(String raw_input);



}

#endif
