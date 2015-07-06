#ifndef STATEACTION_H
#define STATEACTION_H

#include <boost/serialization/access.hpp>

enum Action {UP, DOWN, LEFT, RIGHT}; 

struct State {
    int row;
    int col; 
    int index;
    bool under_sunlight;
    bool has_human; 

    // boost serialize                                                            
    private:                                                                      
      friend class boost::serialization::access;                                  
      template <typename Archive> void serialize(Archive &ar, const unsigned int version) {
        ar & row;                                                                   
        ar & col;                                                                   
      }                                                                           
}; 

struct TerminalState : State {
    bool success; 
}; 

#endif
