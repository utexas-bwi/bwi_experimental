#base.

% DO NOT use room(a ; b). Computation takes forever.

room(l3_414a).
room(l3_402).
room(l3_404).
room(l3_416).
room(l3_418).
room(l3_420).
room(l3_422).
room(l3_430).
room(l3_432).
room(l3_436).
room(l3_428).
room(l3_426).
room(l3_414b).
room(l3_414).
room(l3_412).
room(l3_502).
room(l3_508).
room(l3_510).
room(l3_512).
room(l3_516).



location(l3_400).
location(l3_406).
location(l3_408).
location(l3_410).
location(l3_424).
location(l3_434).
location(l3_500).
location(l3_504).
location(l3_506).
location(l3_520).
location(l3_514).
location(l3_518).


location(R) :- room(R).

door(d3_402).
door(d3_404).
door(d3_412).
door(d3_414a1).
door(d3_414a2).
door(d3_414a3).
door(d3_414b1).
door(d3_414b2).
door(d3_414b3).
door(d3_416).
door(d3_418).
door(d3_420).
door(d3_422).

door(d3_426).
door(d3_428).
door(d3_430).
door(d3_432).
door(d3_436a).
door(d3_436b).

door(d3_502).
door(d3_508).
door(d3_510).
door(d3_512).
door(d3_516a).
door(d3_516b).


hasdoor(l3_402,d3_402).
hasdoor(l3_400,d3_402).
hasdoor(l3_404, d3_404).
hasdoor(l3_400, d3_404).
hasdoor(l3_412, d3_412).
hasdoor(l3_400, d3_412).

hasdoor(l3_414a, d3_414a1).
hasdoor(l3_400, d3_414a1).
hasdoor(l3_414a, d3_414a2).
hasdoor(l3_500, d3_414a2).
hasdoor(l3_414a, d3_414a3).
hasdoor(l3_414, d3_414a3).

hasdoor(l3_414b, d3_414b1).
hasdoor(l3_500, d3_414b1).
hasdoor(l3_414b, d3_414b2).
hasdoor(l3_400, d3_414b2).
hasdoor(l3_414b, d3_414b3).
hasdoor(l3_414, d3_414b3).

hasdoor(l3_416, d3_416).
hasdoor(l3_400, d3_416).
hasdoor(l3_418, d3_418).
hasdoor(l3_400, d3_418).
hasdoor(l3_420, d3_420).
hasdoor(l3_400, d3_420).
hasdoor(l3_422, d3_422).
hasdoor(l3_400, d3_422).


hasdoor(l3_426, d3_426).
hasdoor(l3_400, d3_426).
hasdoor(l3_428, d3_428).
hasdoor(l3_400, d3_428).
hasdoor(l3_430, d3_430).
hasdoor(l3_400, d3_430).
hasdoor(l3_432, d3_432).
hasdoor(l3_400, d3_432).
hasdoor(l3_436, d3_436a).
hasdoor(l3_400, d3_436a).
hasdoor(l3_436, d3_436b).
hasdoor(l3_500, d3_436b).

hasdoor(l3_502, d3_502).
hasdoor(l3_500, d3_502).

hasdoor(l3_508, d3_508).
hasdoor(l3_500, d3_508).
hasdoor(l3_510, d3_510).
hasdoor(l3_500, d3_510).
hasdoor(l3_512, d3_512).
hasdoor(l3_500, d3_512).
hasdoor(l3_514, d3_516a).
hasdoor(l3_516, d3_516a).
hasdoor(l3_516, d3_516b).
hasdoor(l3_500, d3_516b).

acc(l3_406,l3_400).
acc(l3_406,l3_500).
acc(l3_408,l3_400).
acc(l3_410,l3_400).
acc(l3_410,l3_500).
acc(l3_424,l3_400).
acc(l3_434,l3_400).
acc(l3_434,l3_500).
acc(l3_504,l3_500).
acc(l3_506,l3_500).
acc(l3_514,l3_500).
acc(l3_518,l3_500).



dooracc(R1,D,R2) :- hasdoor(R1,D), hasdoor(R2,D), R1 != R2, door(D), room(R1), location(R2).
dooracc(R1,D,R2) :- dooracc(R2,D,R1).

acc(L1,L1) :- location(L1).
acc(L1,L2) :- acc(L2,L1), location(L1), location(L2). 
acc(L1,L2) :- acc(L1,L3), acc(L2,L3), location(L1), location(L2), location(L3).

%you can't be facing a door that is not where you are
:- facing(D,I), at(L,I), not hasdoor(L,D).


%hide non fluents

#hide room/1.
#hide location/1.
#hide door/1.
#hide hasdoor/2.
#hide dooracc/3.
#hide acc/2.
