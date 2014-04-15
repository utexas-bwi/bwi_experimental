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

room(3_414a). % alice
room(3_414). % bob
room(3_414b). % carol
room(3_516). % dan

room(3_400).
room(3_406).
room(3_408).
room(3_410).
room(3_500).
room(3_504).
room(3_506).
room(3_514).
room(3_518).
room(3_520).
room(3_524).
room(3_434).

shop(3_410).

knowinside(alice,3_414a,0).
knowinside(bob,3_414,0).
knowinside(carol,3_414b,0).
knowinside(table1,3_406,0). 
knowinside(coffeecounter,3_410,0). 
%room(R) :- office(R).
room(R) :- shop(R). 

door(3_414a1).
door(3_414a2).
door(3_414j3).
door(3_414b1).
door(3_414b2).
door(3_414b3).
door(3_5161).
door(3_5162).

hasdoor(3_414a,3_414a1).
hasdoor(3_400,3_414a1).
hasdoor(3_414a,3_414a2).
hasdoor(3_500,3_414a2).
hasdoor(3_414a,3_414a3).
hasdoor(3_414,3_414a3).

hasdoor(3_414b,3_414b1).
hasdoor(3_500,3_414b1).
hasdoor(3_414b,3_414b2).
hasdoor(3_400,3_414b2).
hasdoor(3_414b,3_414b3).
hasdoor(3_414,3_414b3).

hasdoor(3_516,3_5161).
hasdoor(3_514,3_5161).
hasdoor(3_516,3_5162).
hasdoor(3_500,3_5162).

item(coffee).
item(bread). 

dooracc(R1,D,R2) :- hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).
dooracc(R1,D,R2) :- dooracc(R2,D,R1), hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).

% cor connectivity
acc(3_400,3_434). 
acc(3_434,3_500). 

% marked open space connectivity
acc(3_400, 3_406).
acc(3_400, 3_408).
acc(3_400, 3_410).
acc(3_400, 3_424).

acc(3_500, 3_504).
acc(3_500, 3_506).
acc(3_500, 3_514).
acc(3_500, 3_518).
acc(3_500, 3_520).

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
