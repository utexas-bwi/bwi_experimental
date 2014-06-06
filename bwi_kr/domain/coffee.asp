%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system laws
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

-beside(D1,I) :- beside(D2,I), D2!=D1, door(D1), door(D2), I=0..n.
-facing(D1,I) :- facing(D2,I), D2!=D1, door(D1), door(D2), I=0..n.
-at(R,I) :- at(R1,I), R1!=R, room(R), room(R1), I=0..n.
beside(D,I) :- facing(D,I), door(D), I=0..n.

-inside(P,R1,I) :- inside(P,R2,I), R2!=R1, room(R1), room(R2), thing(P),
I=0..n.
-knowinside(P,R1,I) :- knowinside(P,R2,I), R2!=R1, room(R1), room(R2),
thing(P), I=0..n.
inside(P,R,I):- knowinside(P,R,I), thing(P), room(R), I=0..n.

facing(D,I+1) :- approach(D,I), door(D), I=0..n-1.
at(R2,I+1) :- approach(D,I), at(R1,I), hasdoor(R2,D), acc(R1,R2), door(D), 
room(R), I=0..n-1. 
-waiting(O,I+1) :- approach(D,I), door(D), item(O), I=0..n-1.
-closeto(P,I+1) :- approach(D,I), door(D), thing(P), I=0..n-1.
:- approach(D,I), facing(D,I), door(D), I=0..n.

at(R,I+1) :- gothrough(D,I), dooracc(R1,D,R), at(R1,I), R1!=R, room(R), door(D),
room(R1), I=0..n-1.
-facing(D,I+1) :- gothrough(D,I), door(D), I=0..n-1.
-waiting(O,I+1) :- gothrough(D,I), door(D), item(O), I=0..n-1.
-closeto(P,I+1) :- gothrough(D,I), door(D), thing(P), I=0..n-1.
:- gothrough(D,I), -facing(D,I), door(D), I=0..n.
:- gothrough(D,I), -open(D,I), door(D), I=0..n.
:- gothrough(D,I), at(R,I), -hasdoor(R,D), door(D), room(R), I=0..n.

at(R,I+1) :- getin(R,I), room(R), I=0..n-1.
-facing(D,I+1) :- getin(R,I), room(R), door(D), I=0..n-1.
-waiting(O,I+1) :- getin(R,I), room(R), item(O), I=0..n-1.
-closeto(P,I+1) :- getin(R,I), room(R), thing(P), I=0..n-1.
:- getin(R,I), at(R1,I), -acc(R,R1), room(R), room(R1), R!=R1, I=0..n. 

open(D,I+1) :- opendoor(D,I), door(D), I=0..n-1.
:- opendoor(D,I), -facing(D,I), door(D), I=0..n.
:- opendoor(D,I), open(D,I), door(D), I=0..n.

closeto(P,I+1) :- greet(P,I), person(P), I=0..n-1.
closeto(O,I+1) :- goto(O,I), object(O), I=0..n-1.
at(R,I+1) :- goto(O,I), knowinside(O,R,I), object(O), room(R), I=0..n-1.
-facing(D,I+1) :- greet(P,I), person(P), door(D), I=0..n-1.
-waiting(IT,I+1) :- greet(P,I), person(P), item(IT), I=0..n-1.
-facing(D,I+1) :- goto(O,I), object(O), door(D), I=0..n-1.
-beside(D,I+1) :- goto(O,I), object(O), door(D), I=0..n-1.
-waiting(IT,I+1) :- goto(O,I), object(O), item(IT), I=0..n-1.
-closeto(T1,I) :- closeto(T2,I), thing(T1), thing(T2), T1!=T2, I=0..n.
:- greet(P,I), at(R,I), -knowinside(P,R,I), person(P), room(R), I=0..n.
:- greet(P,I), -at(R,I), knowinside(P,R,I), person(P), room(R), I=0..n.
:- goto(O,I), -at(R,I), at(R2,I), -acc(R,R2), knowinside(O,R,I), object(O), room(R), room(R2), R!=R2, I=0..n.
%:- goto(O,I), at(R,I), -knowinside(O,R,I), object(O), room(R), I=0..n.

knowinside(P,R,I+1) :- askploc(P1,P,I), inside(P,R,I), person(P), person(P1),
room(R), I=0..n-1.
:- askploc(P1,P,I), -closeto(P1,I), I=0..n.
:- askploc(P1,P,I), closeto(P1,I), -knows(P1,P), I=0..n.
:- askploc(P1,P,I), knowinside(P,R,I), I=0..n.

waiting(O,I+1) :- order(O,I), item(O), I=0..n-1.
:- order(O,I), item(O), at(R,I), -shop(R), I=0..n.
:- order(O,I), item(O), at(R,I), shop(R), {closeto(T,I):thing(T)}0, I=0..n.
:- order(O,I), item(O), at(R,I), shop(R), closeto(SC,I), -shopcounter(SC), I=0..n.

loaded(O,I+1) :- load(O,I), item(O), I=0..n-1. 
-waiting(O1,I+1) :- load(O,I), item(O1), I=0..n-1. 
:- load(O,I), -waiting(O,I), item(O), I=0..n. 
:- load(O,I), loaded(O1,I), item(O), item(O1), I=0..n. 

served(P,O,I+1) :- unloadto(O,P,I), item(O), thing(P), I=0..n-1. 
-loaded(O,I+1) :- unloadto(O,P,I), thing(P), I=0..n-1. 
:- unloadto(O,P,I), -closeto(P,I), I=0..n. 
:- unloadto(O,P,I), -loaded(O,I), I=0..n. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% inertial laws
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

at(R,I+1) :- at(R,I), not -at(R,I+1), I=0..n-1.
-at(R,I+1) :- -at(R,I), not at(R,I+1), I=0..n-1.

inside(P,R,I+1) :- inside(P,R,I), not -inside(P,R,I+1), I=0..n-1.
-inside(P,R,I+1) :- -inside(P,R,I), not inside(P,R,I+1), I=0..n-1.

beside(D,I+1):- beside(D,I), not -beside(D,I+1), I=0..n-1.
-beside(D,I+1):- -beside(D,I), not beside(D,I+1), I=0..n-1.

facing(D,I+1) :- facing(D,I), not -facing(D,I+1), I=0..n-1.
-facing(D,I+1) :- -facing(D,I), not facing(D,I+1), I=0..n-1.

knowinside(P,R,I+1) :- knowinside(P,R,I), not -knowinside(P,R,I+1),
    I=0..n-1.
-knowinside(P,R,I+1) :- -knowinside(P,R,I), not knowinside(P,R,I+1),
    I=0..n-1.

served(P,O,I+1) :- served(P,O,I), not -served(P,O,I+1), I=0..n-1.
-served(P,O,I+1) :- -served(P,O,I), not served(P,O,I+1), I=0..n-1.

waiting(O,I+1) :- waiting(O,I), not -waiting(O,I+1), I=0..n-1. 
-waiting(O,I+1) :- -waiting(O,I), not waiting(O,I+1), I=0..n-1. 

loaded(O,I+1) :- loaded(O,I), not -loaded(O,I+1), I=0..n-1. 
-loaded(O,I+1) :- -loaded(O,I), not loaded(O,I+1), I=0..n-1. 

closeto(P,I+1) :- closeto(P,I), not -closeto(P,I+1), I=0..n-1. 
-closeto(P,I+1) :- -closeto(P,I), not closeto(P,I+1), I=0..n-1. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% exogenous rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

{approach(D,I)} :- door(D), I=0..n-1.
{gothrough(D,I)} :- door(D), I=0..n-1.
{opendoor(D,I)} :- door(D), I=0..n-1.
{greet(P,I)} :- person(P), I=0..n-1.
{goto(O,I)} :- object(O), I=0..n-1.
{askploc(P1,P2,I)} :- thing(P1), thing(P2), I=0..n-1.
{order(O,I)} :- item(O), I=0..n-1. 
{load(O,I)} :- item(O), I=0..n-1. 
{unloadto(O,P,I)} :- item(O), thing(P), I=0..n-1. 
{getin(R,I)} :- room(R), I=0..n-1. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nonexecutabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- getin(R,I), load(O1,I).
:- getin(R,I), unloadto(O1,P,I).
:- getin(R,I), approach(D,I). 
:- getin(R,I), gothrough(D,I).
:- getin(R,I), opendoor(D,I). 
:- getin(R,I), greet(P,I). 
:- getin(R,I), askploc(P1,P2,I).
:- getin(R,I), order(IT,I). 
:- getin(R,I), goto(O,I).
:- goto(O,I), load(O1,I).
:- goto(O,I), unloadto(O1,P,I).
:- goto(O,I), approach(D,I). 
:- goto(O,I), gothrough(D,I).
:- goto(O,I), opendoor(D,I). 
:- goto(O,I), greet(P,I).
:- goto(O,I), askploc(P1,P2,I).
:- goto(O,I), order(IT,I). 
:- order(O,I), load(O1,I).
:- order(O,I), unloadto(O1,P,I).
:- order(O,I), approach(D,I). 
:- order(O,I), gothrough(D,I).
:- order(O,I), opendoor(D,I). 
:- order(O,I), greet(P,I). 
:- order(O,I), askploc(P1,P2,I).
:- load(O,I), unloadto(O1,P,I).
:- load(O,I), approach(D,I).
:- load(O,I), gothrough(D,I).
:- load(O,I), opendoor(D,I).
:- load(O,I), greet(P,I).
:- load(O,I), askploc(P1,P2,I).
:- unloadto(O,T,I), approach(D,I).
:- unloadto(O,T,I), gothrough(D,I).
:- unloadto(O,T,I), opendoor(D,I).
:- unloadto(O,T,I), greet(P,I).
:- unloadto(O,T,I), askploc(P1,P2,I).
:- approach(D1,I), gothrough(D2,I).
:- approach(D1,I), opendoor(D2,I).
:- approach(D,I), greet(P,I).
:- approach(D,I), askploc(P1,P2,I).
:- gothrough(D,I), opendoor(D1,I).
:- gothrough(D,I), greet(P,I).
:- gothrough(D,I), askploc(P1,P2,I).
:- opendoor(D,I), greet(P,I).
:- opendoor(D,I), askploc(P1,P2,I).
:- greet(P,I), askploc(P1,P2,I).

:- noop(I), approach(D,I).
:- noop(I), gothrough(D,I).
:- noop(I), opendoor(D,I).
:- noop(I), greet(P,I).
:- noop(I), goto(O,I).
:- noop(I), askploc(P1,P2,I).
:- noop(I), load(O,I).
:- noop(I), unloadto(O,P,I).
:- noop(I), order(IT,I).
:- noop(I), getin(R,I).

:- approach(D1,I), approach(D2,I), D1 != D2.
:- order(O1,I), order(O2,I), O1!=O2.
:- load(O1,I), load(O2,I), O1!=O2. 
:- unloadto(O1,P,I), unloadto(O2,P,I), O1!=O2. 
:- unloadto(O,P1,I), unloadto(O,P2,I), P1!=P2. 
:- opendoor(D1,I), opendoor(D2,I), D1!=D2.
:- greet(P1,I), greet(P2,I), P1!=P2.
:- goto(O1,I), goto(O2,I), O1!=O2.
:- askploc(P1,P,I), askploc(P2,P,I), P1!=P2.
:- askploc(P,P1,I), askploc(P,P2,I), P1!=P2.
:- getin(R1,I), getin(R2,I), R1!=R2. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial choice rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{inside(P,R,0)} :- thing(P),room(R).
%{beside(D,0)} :- door(D).
%{facing(D,0)} :- door(D).
%{knowinside(P,R,0)} :- thing(P), room(R).
%{served(P,O,0)} :- thing(P), item(O).
%{at(R,0)} :- room(R).
%{waiting(O,0)} :- item(O). 
%{loaded(O,0)} :- item(O). 
%{closeto(P,0)} :- thing(P).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% completeness
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%:- not inside(P,R,I), not -inside(P,R,I), thing(P), room(R), I=0..n.
%:- not beside(D,I), not -beside(D,I), door(D), I=0..n.
%:- not facing(D,I), not -facing(D,I), door(D), I=0..n.
%:- not knowinside(P,R,I), not -knowinside(P,R,I), thing(P), room(R), I=0..n.
%:- not served(P,O,I), not -served(P,O,I), thing(P), item(O), I=0..n.
%:- not at(R,I), not -at(R,I), room(R), I=0..n.
%:- not waiting(O,I), not -waiting(O,I), item(O), I=0..n. 
%:- not loaded(O,I), not -loaded(O,I), item(O), I=0..n. 
%:- not closeto(T,I), not -closeto(T,I), thing(T), I=0..n. 

%#hide.
%#show knowinside/3.
%#show -knowinside/3.
%#show inside/3.
%#show -inside/3.
%#show beside/2.
%#show -beside/2.
%#show facing/2.
%#show -facing/2.
%#show served/3.
%#show -served/3.
%#show at/2.
%#show -at/2.
%#show closeto/2.
%#show -closeto/2.
%#show served/3.
%#show -served/3.
%#show loaded/2.
%#show -loaded/2.
%#show open/2.
%#show -open/2.
%#show waiting/2.
%#show -waiting/2.

%#show approach/2.
%#show approach/2.
%#show gothrough/2.
%#show opendoor/2.
%#show greet/2.
%#show goto/2.
%#show askploc/3.
%#show load/2.
%#show unloadto/3.
%#show order/2.
%#show getin/2.
%#show noop/1.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial values (unless specified otherwise)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

-served(P,O,0) :- not served(P,O,0), thing(P), item(O).
-waiting(O,0) :- not waiting(O,0), item(O).
-loaded(O,0) :- not loaded(O,0), item(O).
-closeto(P,0) :- not closeto(P,0), thing(P).
