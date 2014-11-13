person(peter). 
person(ray). 
person(dana). 
person(kazunori). 
person(matteo). 
person(shiqi). 
person(jivko). 
person(stacy).

hasoffice(peter,l3_508). 
hasoffice(ray,l3_512).
hasoffice(dana,l3_510). 
hasoffice(kazunori,l3_402). 
hasoffice(matteo,l3_418).
hasoffice(shiqi,l3_420).
hasoffice(jivko,l3_432). 
hasoffice(stacy,l3_502). 

canbeinroom(P,R) :- hasoffice(P,R), person(P), room(R).

canbeinroom(matteo, l3_414b).
canbeinroom(shiqi, l3_414b).
canbeinroom(jivko, l3_414b).

canknow(P1,P2) :- canknow(P1,P2).

#hide person/1.
#hide hasoffice/2.
#hide canbeinroom/2.

