knowinroom(P,R,I+1) :- askinroom(P,R,I), person(P), room(R), I=0..n-1.
1{ inroom(P,R,I), -inroom(P,R,I) }1 :- knowinroom(P,R,I), person(P), room(R), I=0..n.
-inroom(P,R1,I) :- inroom(P,R2,I), R1 != R2, person(P), room(R1), room(R2), I=0..n.
knowinoffice(P,I) :- knowinroom(P,R,I), inside(P,R), person(P), room(R), I=0..n.
:- askinroom(P,R,I), not facing(D,I), not at(R,I), hasdoor(R,D), person(P), room(R), door(D), I=0..n.
:- askinroom(P,R,I), facing(D,I), open(D,I), hasdoor(R,D), person(P), room(R), door(D), I=0..n.

#hide -open/2.
#hide beside/2.
#hide -inroom/3.
