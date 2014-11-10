%action askinroom(P,R,I)  ask if person P is in room R
1{ inroom(P,R,I+1), -inroom(P,R,I+1) }1 :- askinroom(P,R,I), person(P), room(R), I=0..n-1.
:- askinroom(P,R,I), not facing(D,I) : hasdoor(R,D), not at(R,I), person(P), room(R), I=0..n.
:- askinroom(P,R,I), facing(D,I), open(D,I), hasdoor(R,D), person(P), room(R), door(D), I=0..n.
:- askinroom(P,R,I), inroom(P,R,I), person(P), room(R), I=0..n.
:- askinroom(P,R,I), -inroom(P,R,I), person(P), room(R), I=0..n.
:- askinroom(P,R,I), not canbeinroom(P,R), person(P), room(R), I=0..n.

%inroom is inertial
inroom(P,R,I+1) :- inroom(P,R,I), not -inroom(P,R,I+1), I=0..n-1.
-inroom(P,R,I+1) :- -inroom(P,R,I), not inroom(P,R,I+1), I=0..n-1.
:- inroom(P,R,I), not room(R).

%fluent inoffice(P,I)
inoffice(P,I) :- inroom(P,R,I), hasoffice(P,R), person(P), room(R), I=0..n.
-inoffice(P,I) :- -inroom(P,R,I), hasoffice(P,R), person(P), room(R), I=0..n.

%fluent ingdc(P,I)
ingdc(P,I) :- inroom(P,R,I), person(P), room(R), I=0..n.
-ingdc(P,I) :- 0{not -inroom(P,R,I) : canbeinroom(P,R)}0, 0{not -know(P1,P,I) : canknow(P1,P)}0, person(P), I=0..n.

%action askpsnroom(P1,P2,I)  ask P1 where P2 is
1{inroom(P2,R,I+1) : room(R)}1 :- askpsnroom(P1,P2,I), person(P1), person(P2), I=0..n-1.
:- askpsnroom(P1,P2,I), not inroom(P1,R,I) : room(R), at(R,I), person(P1), person(P2), I=0..n.
:- askpsnroom(P1,P2,I), inroom(P2,R,I), person(P1), person(P2), room(R), I=0..n.
:- askpsnroom(P1,P2,I), not canknow(P1,P2), person(P1), person(P2), I=0..n.

%fluent know(P1,P2)  P1 knows where P2 is
1{know(P1,P2,I+1), -know(P1,P2,I+1)}1 :- askpsnroom(P1,P2,I), person(P1), person(P2), I=0..n-1.
-know(P1, P2, I) :- -ingdc(P1, I), canknow(P1,P2), person(P1), person(P2), I=0..n.

%know is inertial
know(P1,P2,I+1) :- know(P1,P2,I), not -know(P1,P2,I+1), I=0..n-1.
-know(P1,P2,I+1) :- -know(P1,P2,I), not know(P1,P2,I+1), I=0..n-1.
