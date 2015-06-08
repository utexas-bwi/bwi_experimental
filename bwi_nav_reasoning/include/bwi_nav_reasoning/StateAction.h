
#ifndef STATEACTION_H
#define STATEACTION_H

enum Action {UP, DOWN, LEFT, RIGHT}; 

struct State {
    int row;
    int col; 
    int index;
    bool under_sunlight;
    bool has_human; 
}; 

struct TerminalState : State {
    bool success; 
}; 

#endif
