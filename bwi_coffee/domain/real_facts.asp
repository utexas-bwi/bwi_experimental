%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% facts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

thing(P) :- person(P).
thing(O) :- object(O). 
object(SC) :- shopcounter(SC).

person(alice).
person(bob).
person(carol).
person(dan).

object(table1). 
shopcounter(coffeecounter). 

room(r3_414a). % alice
room(r3_414). % bob
room(r3_414b). % carol
room(r3_516). % dan

room(r3_400).
room(r3_406).
room(r3_408).
room(r3_410).
room(r3_500).
room(r3_504).
room(r3_506).
room(r3_514).
room(r3_518).
room(r3_520).
room(r3_524).
room(r3_434).

shop(r3_410).

knowinside(alice,r3_414a,0).
knowinside(bob,r3_414,0).
knowinside(carol,r3_414b,0).
knowinside(table1,r3_406,0). 
knowinside(coffeecounter,r3_410,0). 
%room(R) :- office(R).
room(R) :- shop(R). 

door(d3_414a1).
door(d3_414a2).
door(d3_414j3).
door(d3_414b1).
door(d3_414b2).
door(d3_414b3).
door(d3_5161).
door(d3_5162).

hasdoor(r3_414a,d3_414a1).
hasdoor(r3_400,d3_414a1).
hasdoor(r3_414a,d3_414a2).
hasdoor(r3_500,d3_414a2).
hasdoor(r3_414a,d3_414a3).
hasdoor(r3_414,d3_414a3).

hasdoor(r3_414b,d3_414b1).
hasdoor(r3_500,d3_414b1).
hasdoor(r3_414b,d3_414b2).
hasdoor(r3_400,d3_414b2).
hasdoor(r3_414b,d3_414b3).
hasdoor(r3_414,d3_414b3).

hasdoor(r3_516,d3_5161).
hasdoor(r3_514,d3_5161).
hasdoor(r3_516,d3_5162).
hasdoor(r3_500,d3_5162).

item(coffee).
item(bread). 

dooracc(R1,D,R2) :- hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).
dooracc(R1,D,R2) :- dooracc(R2,D,R1), hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).

% cor connectivity
acc(r3_400,r3_434). 
acc(r3_434,r3_500). 

% marked open space connectivity
acc(r3_400, r3_406).
acc(r3_400, r3_408).
acc(r3_400, r3_410).
acc(r3_400, r3_424).

acc(r3_500, r3_504).
acc(r3_500, r3_506).
acc(r3_500, r3_514).
acc(r3_500, r3_518).
acc(r3_500, r3_520).

acc(R,R) :- room(R). 

acc(R1,R2) :- acc(R2,R1), room(R1), room(R2). 
acc(R1,R2) :- acc(R1,R3), acc(R2,R3), room(R1), room(R2), room(R3).

knows(alice, dan).

-hasdoor(R,D) :- not hasdoor(R,D), room(R), door(D).
-open(D,I) :- not open(D,I), door(D), I=0..n.
-dooracc(R1,D,R2) :- not dooracc(R1,D,R2), room(R1), room(R2), room(D).
-acc(R1,R2) :- not acc(R1,R2), room(R1), room(R2). 
-knows(P1,P2) :- not knows(P1,P2), person(P1), person(P2).
-shop(R) :- not shop(R), room(R). 
-shopcounter(SC) :- not shopcounter(SC), thing(SC). 
