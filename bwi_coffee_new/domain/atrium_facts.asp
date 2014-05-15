%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% facts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

thing(P) :- person(P).
thing(O) :- object(O). 
object(SC) :- shopcounter(SC).

person(peter).

object(table1). 
object(table2). 
shopcounter(coffeecounter). 

room(atrium).
room(cor1).
room(cor2).
room(cor3).
room(openarea).
room(elevator).

shop(coffeeshop).

knowinside(peter,elevator,0). 
knowinside(table1,atrium,0). 
knowinside(table2,atrium,0). 
knowinside(coffeecounter,coffeeshop,0). 

room(R) :- shop(R). 

door(d_elevator).

hasdoor(elevator,d_elevator).
hasdoor(cor3,d_elevator).

item(coffee).
item(mocha). 

dooracc(R1,D,R2) :- hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).
dooracc(R1,D,R2) :- dooracc(R2,D,R1), hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).

% cor connectivity
acc(atrium,cor1). 
acc(atrium,cor2). 
acc(atrium,coffeeshop). 
acc(atrium,openarea). 
acc(cor2,cor3). 

acc(R,R) :- room(R). 

acc(R1,R2) :- acc(R2,R1), room(R1), room(R2). 
acc(R1,R2) :- acc(R1,R3), acc(R2,R3), room(R1), room(R2), room(R3).

-hasdoor(R,D) :- not hasdoor(R,D), room(R), door(D).
-open(D,I) :- not open(D,I), door(D), I=0..n.
-dooracc(R1,D,R2) :- not dooracc(R1,D,R2), room(R1), room(R2), room(D).
-acc(R1,R2) :- not acc(R1,R2), room(R1), room(R2). 
-knows(P1,P2) :- not knows(P1,P2), person(P1), person(P2).
-shop(R) :- not shop(R), room(R). 
-shopcounter(SC) :- not shopcounter(SC), thing(SC). 
