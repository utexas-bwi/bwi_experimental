#cumulative n.

%you have to pick exactly one action

%LIST HERE ANY ACTION YOU ADD!!!

1{	approach(D1,I) : door(D1), 
	gothrough(D2,I) : door(D2), 
	opendoor(D3,I) : door(D3), 
        searchroom(P,R,I): person(P) : room(R),
        askperson(P1,P2,I): person(P1) : person(P2),
        remind(P,M,R,I) : person(P) : meeting(M,G,R) : room(R)
	}1 :- not noop(I), I=0..n-2.

%removes the warning about noop not being defined, shouldn't have any consequences

%iclingo doesn't seem to like this, commenting.
%#volatile n.
%noop(n).

#hide noop/1.

