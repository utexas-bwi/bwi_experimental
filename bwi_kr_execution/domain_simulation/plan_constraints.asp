% add constraints to avoid stupid plans, but make sure you don't exclude the ones you want.

:- approach(D,I), open(D,I), not gothrough(D,I+1).
:- approach(D,I), not open(D,I), not gothrough(D,I+2).
:- opendoor(D,I), not gothrough(D,I+1).