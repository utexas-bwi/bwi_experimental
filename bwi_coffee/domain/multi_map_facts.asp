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

floor(f1). 
floor(f2). 
floor(f3). 

%------Floor1
room(f3_410).
%shop(f3_504).
room(f3_504).
room(f3_500).
room(f3_200).
%------Floor2
room(f2_main).
room(f2_corr2).
room(f2_corr3).
room(f2_side).
room(f2_drink).
shop(f2_coffee).
room(f2_main_plus).

%elevator(f1_ele1). 
elevator(f2_ele1). 
elevator(f3_ele1). 

%------Floor1
%onfloor(f1_ele1,f1).
onfloor(f2_ele1,f2).
onfloor(f3_ele1,f3).
onfloor(f3_410,f3).
onfloor(f3_504,f3).
onfloor(f3_500,f3).
onfloor(f3_200,f3).
%------Floor2
onfloor(f2_main,f2).
onfloor(f2_corr2,f2).
onfloor(f2_corr3,f2).
onfloor(f2_side,f2).
onfloor(f2_drink,f2).
onfloor(f2_coffee,f2).
onfloor(f2_main_plus,f2).


knowinside(alice,f2_ele1,0).
%knowinside(alice,f1_atrium,0).
%knowinside(bob,o3_510,0).
%knowinside(carol,o3_512,0).
knowinside(table1,f3_410,0). 
%knowinside(coffeecounter,f3_504,0). 
knowinside(coffeecounter,f2_coffee,0). 

room(R) :- office(R).
room(R) :- shop(R). 
room(R) :- elevator(R). 

%------Floor1
door(d3_500).
door(d3_ele1).
%------Floor2
door(d2_ele1).

%------Floor1
hasdoor(f3_ele1,d3_ele1).
hasdoor(f3_200,d3_ele1).
hasdoor(f3_500,d3_500).
hasdoor(f3_200,d3_500).
%------Floor2
hasdoor(f2_ele1,d2_ele1).
hasdoor(f2_corr3, d2_ele1).

%------Floor1
acc(f3_500,f3_504). 
acc(f3_500,f3_410). 
%------Floor2
acc(f2_main,f2_corr2). 
acc(f2_main,f2_main_plus). 
acc(f2_main,f2_drink). 
acc(f2_main,f2_side). 
acc(f2_drink,f2_coffee). 
acc(f2_corr2,f2_corr3). 
acc(f2_side,f2_corr3). 

item(coffee).
item(bread). 

thingonfloor(T,F) :- at(T,R), onfloor(R,F), room(R), thing(T). 
dooronfloor(D,F) :- hasdoor(R,D), onfloor(R,F), room(R), door(D). 

dooracc(R1,D,R2) :- hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).
dooracc(R1,D,R2) :- dooracc(R2,D,R1), hasdoor(R1,D), hasdoor(R2,D), room(R1), room(R2), door(D).

acc(f3_410,f3_cor4). 
acc(f3_410,f3_cor5). 
acc(f3_cor5,f3_504). 
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
-onfloor(R,F) :- not onfloor(R,F), room(R), floor(F). 
-thingonfloor(T,F) :- not thingonfloor(T,F), thing(T), floor(F). 
-dooronfloor(D,F) :- not dooronfloor(D,F), door(D), floor(F). 
-elevator(E) :- not elevator(E), room(E). 
