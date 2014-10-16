visited(R,I) :- at(R,I), room(R). 
visited(R,I+1) :- visited(R,I), I=0..n-1.