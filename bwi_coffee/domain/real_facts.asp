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

room(l3_414a). % alice
room(l3_414). % bob
room(l3_414b). % carol
room(l3_516). % dan

room(l3_400).
room(l3_406).
room(l3_408).
room(l3_410).
room(l3_500).
room(l3_504).
room(l3_506).
room(l3_514).
room(l3_518).
room(l3_520).
room(l3_524).
room(l3_434).

shop(l3_410).

knowinside(alice,l3_414a,0).
knowinside(bob,l3_414,0).
knowinside(carol,l3_414b,0).
knowinside(table1,l3_406,0). 
knowinside(coffeecounter,l3_410,0). 
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

hasdoor(l3_414a,d3_414a1).
hasdoor(l3_400,d3_414a1).
hasdoor(l3_414a,d3_414a2).
hasdoor(l3_500,d3_414a2).
hasdoor(l3_414a,d3_414a3).
hasdoor(l3_414,d3_414a3).

hasdoor(l3_414b,d3_414b1).
hasdoor(l3_500,d3_414b1).
hasdoor(l3_414b,d3_414b2).
hasdoor(l3_400,d3_414b2).
hasdoor(l3_414b,d3_414b3).
hasdoor(l3_414,d3_414b3).

hasdoor(l3_516,d3_5161).
hasdoor(l3_514,d3_5161).
hasdoor(l3_516,d3_5162).
hasdoor(l3_500,d3_5162).

item(coffee).
item(bread). 

dooracc(R1,D,R2) :- hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).
dooracc(R1,D,R2) :- dooracc(R2,D,R1), hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).

% cor connectivity
acc(l3_400,l3_434). 
acc(l3_434,l3_500). 

% marked open space connectivity
acc(l3_400, l3_406).
acc(l3_400, l3_408).
acc(l3_400, l3_410).
acc(l3_400, l3_424).

acc(l3_500, l3_504).
acc(l3_500, l3_506).
acc(l3_500, l3_514).
acc(l3_500, l3_518).
acc(l3_500, l3_520).

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
