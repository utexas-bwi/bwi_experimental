%here I'm trying to avoid at least some stupid plans, not sure it makes sense.

:- approach(D,I), open(D,I), not gothrough(D,I+1).
:- approach(D,I), not open(D,I), not gothrough(D,I+2).

:- at(L1, I1), at(R1, I2), room(R1), not room(L1),  at(L1,I3), I2 < I3, I1 < I2.