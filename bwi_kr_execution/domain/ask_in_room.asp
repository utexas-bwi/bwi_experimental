1{ inroom(P,R,I+1), -inroom(P,R,I+1) }1 :- askinroom(P,R,I), person(P), room(R), I=0..n-1.
inoffice(P,I) :- inroom(P,R,I), inside(P,R), person(P), room(R), I=0..n.
-inoffice(P,I) :- -inroom(P,R,I), inside(P,R), person(P), room(R), I=0..n.
:- askinroom(P,R,I), not facing(D,I) : hasdoor(R,D) : door(D), not at(R,I), person(P), room(R), I=0..n.
:- askinroom(P,R,I), facing(D,I), open(D,I), hasdoor(R,D), person(P), room(R), door(D), I=0..n.
:- askinroom(P,R,I), inroom(P,R,I), person(P), room(R), I=0..n.
:- askinroom(P,R,I), -inroom(P,R,I), person(P), room(R), I=0..n.

%inertial laws
inroom(P,R,I+1) :- inroom(P,R,I), not -inroom(P,R,I+1), I=0..n-1.
-inroom(P,R,I+1) :- -inroom(P,R,I), not inroom(P,R,I+1), I=0..n-1.

%somewhere or nowhere
inGDC(P,I) :- inroom(P,R,I), person(P), room(R), I=0..n.
-inGDC(P,I):- 0{not -inroom(P,R,I) : room(R)}0, person(P), I=0..n.

