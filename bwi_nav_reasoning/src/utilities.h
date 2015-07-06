#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

string getStdoutFromCommand(string cmd) {

    string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");
    
    stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
        if (fgets(buffer, max_buffer, stream) != NULL) 
            data.append(buffer);
        pclose(stream);
    }
    return data;
}


#endif
