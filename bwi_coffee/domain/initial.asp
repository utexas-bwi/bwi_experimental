%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial state (for testing)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

at(cor,0).
-facing(D,0) :- door(D).
-beside(D,0) :- door(D).
-visiting(P,0) :- person(P).
-served(P,O,0) :- person(P), item(O).
-waiting(O,0) :- item(O). 
-loaded(O,0) :- item(O). 
-closeto(P,0) :- person(P). 
