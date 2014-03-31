%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% facts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

person(alice).
person(bob).
person(carol).
person(dan).

knowinside(alice,o3_508,0).
knowinside(bob,o3_510,0).
knowinside(carol,o3_512,0).

office(o3_508). % alice
office(o3_510). % bob
office(o3_512). % carol
office(o3_428).
office(o3_426).

room(cor).
room(o3_502).

room(R) :- office(R).

door(d3_508).
door(d3_510).
door(d3_512).
door(d3_428).
door(d3_426).
door(d3_502).

hasdoor(o3_508,d3_508).
hasdoor(o3_510,d3_510).
hasdoor(o3_512,d3_512).
hasdoor(o3_426,d3_426).
hasdoor(o3_428,d3_428).
hasdoor(cor,D) :- door(D).
hasdoor(o3_502,d3_502) :- door(d3_502).
-hasdoor(R,D) :- not hasdoor(R,D), room(R), door(D).

item(coffee).
item(bread). 

-open(D,I) :- not open(D,I), door(D), I=0..n.
-visiting(P,I) :- not visiting(P,I), door(P), I=0..n.

-acc(R1,D,R2) :- not acc(R1,D,R2), room(R1), room(R2), room(D).
acc(R,D,cor) :- hasdoor(R,D), R!=cor.
acc(R1,D,R2) :- acc(R2,D,R1).

acc(l_414a,d_414a3,l_414).
acc(l_414b,d_414b3,l_414).

knows(alice,dan).
-knows(P1,P2) :- not knows(P1,P2), person(P1), person(P2).
