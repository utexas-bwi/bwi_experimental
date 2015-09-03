#include "StateAction.h"

std::ostream& operator<<(std::ostream& stream, const Action& action) {          
  if (action == UP) {                                                           
    stream << "Up";                                                             
  } else if (action == DOWN) {                                                  
    stream << "Down";                                                           
  } else if (action == LEFT) {                                                  
    stream << "Left";                                                           
  } else {                                                                      
    stream << "Right";                                                          
  }
  return stream;
}
 
std::ostream& operator<<(std::ostream& stream, const State& s) {                
  stream << "(" << s.row << "," << s.col << ")";
  return stream;                                                                
}                                                                               

bool operator<(const State& l, const State& r) {
    if (l.row < r.row) return true;
    if (l.row > r.row) return false;

    if (l.col < r.col) return true;
    if (l.col > r.col) return false;
    return false; 
}
                                                                                
bool operator==(const State& l, const State& r) {                               
    return (l.row == r.row && l.col == r.col);                                            
}                                                                               


