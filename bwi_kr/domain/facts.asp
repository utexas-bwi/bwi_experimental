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

office(o3_508). % alice
office(o3_510). % bob
office(o3_512). % carol
office(o3_428).
office(o3_426).

room(f3_410).
room(cor).
room(cor).
room(l3_414b).
room(l3_414).
room(l3_414a).
shop(f3_504).

room(R) :- office(R).
room(R) :- shop(R). 

door(d3_508).
door(d3_510).
door(d3_512).
door(d3_428).
door(d3_426).
door(d3_414b1).
door(d3_414b2).
door(d3_414b3).

hasdoor(o3_508,d3_508).
hasdoor(o3_510,d3_510).
hasdoor(o3_512,d3_512).
hasdoor(o3_426,d3_426).
hasdoor(o3_428,d3_428).

hasdoor(cor,d3_426).
hasdoor(cor,d3_428).
hasdoor(cor,d3_508).
hasdoor(cor,d3_510).
hasdoor(cor,d3_512).

hasdoor(l3_414b,d3_414b1).
hasdoor(l3_414b,d3_414b2).
hasdoor(l3_414b,d3_414b3).
hasdoor(l3_414,d3_414b3).
hasdoor(cor,d3_414b2).
hasdoor(cor,d3_414b1).

item(coffee).
item(bread). 

dooracc(R1,D,R2) :- hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).
dooracc(R1,D,R2) :- dooracc(R2,D,R1), hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).

acc(f3_410,cor). 
acc(f3_410,cor). 
acc(cor,f3_504). 
acc(R1,R2) :- acc(R2,R1), room(R1), room(R2). 
acc(R1,R2) :- acc(R1,R3), acc(R2,R3), room(R1), room(R2), room(R3).

knows(alice, dan).

-hasdoor(R,D) :- not hasdoor(R,D), room(R), door(D).
-dooracc(R1,D,R2) :- not dooracc(R1,D,R2), room(R1), room(R2), room(D).
-acc(R1,R2) :- not acc(R1,R2), room(R1), room(R2). 
-knows(P1,P2) :- not knows(P1,P2), person(P1), person(P2).
-shop(R) :- not shop(R), room(R). 
-shopcounter(SC) :- not shopcounter(SC), thing(SC). 
