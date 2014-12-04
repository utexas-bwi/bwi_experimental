//wh-words
what :- S/(S\NP)/N : (lambda $0:<e,t> (lambda $1:<e,t> (lambda $2:e (and:<t*,t> ($0 $2) ($1 $2)))))
what :- S/(S/NP)/N : (lambda $0:<e,t> (lambda $1:<e,t> (lambda $2:e (and:<t*,t> ($0 $2) ($1 $2)))))
how many :- S/(S\NP)/N : (lambda $0:<e,t> (lambda $1:<e,t> (count:<<e,t>,i> (lambda $2:e (and:<t*,t> ($0 $2) ($1 $2))))))
how many :- S/(S/NP)/N : (lambda $0:<e,t> (lambda $1:<e,t> (count:<<e,t>,i> (lambda $2:e (and:<t*,t> ($0 $2) ($1 $2))))))
count :- S/(S\NP)/N : (lambda $0:<e,t> (lambda $1:<e,t> (count:<<e,t>,i> (lambda $2:e (and:<t*,t> ($0 $2) ($1 $2))))))
count :- S/(S/NP)/N : (lambda $0:<e,t> (lambda $1:<e,t> (count:<<e,t>,i> (lambda $2:e (and:<t*,t> ($0 $2) ($1 $2))))))
which :- S/NP : (lambda $0:e $0)
how many :- S/NP : (lambda $0:e $0)
how :- S/NP : (lambda $0:e $0)
what :- S/NP : (lambda $0:e $0)
where :- S/NP : (lambda $0:e $0)

// commands
take :- (S/NP)/NP : (lambda $0:e (lambda $1:e (and:<t*,t> (action:<a,t> bring:a) (actionrecipient:<a,<e,t>> bring:a $0) (actionpatient:<a,<e,t>> bring:a $1))))
take :- (S/PP)/NP : (lambda $0:e (lambda $1:<a,t> (and:<t*,t> (action:<a,t> bring:a) (actionpatient:<a,<e,t>> bring:a $0) ($1 bring:a))))
bring :- (S/NP)/NP : (lambda $0:e (lambda $1:e (and:<t*,t> (action:<a,t> bring:a) (actionrecipient:<a,<e,t>> bring:a $0) (actionpatient:<a,<e,t>> bring:a $1))))
bring :- (S/PP)/NP : (lambda $0:e (lambda $1:<a,t> (and:<t*,t> (action:<a,t> bring:a) (actionpatient:<a,<e,t>> bring:a $0) ($1 bring:a))))

walk :- S/PP : (lambda $0:<a,t> (and:<t*,t> (action:<a,t> walk:a) ($0 walk:a)))
go :- S/PP : (lambda $0:<a,t> (and:<t*,t> (action:<a,t> walk:a) ($0 walk:a)))

to :- PP/NP : (lambda $0:e (lambda $1:a (actionrecipient:<a,<e,t>> $1 $0)))

// nouns
who :- N : person:<pe,t>
person :- N : person:<pe,t>
people :- N : person:<pe,t>
object :- N : object:<ob,t>
objects :- N : object:<ob,t>
room :- N : room:<ro,t>
rooms :- N : room:<ro,t>
office :- N : office:<of,t>
offices :- N : office:<of,t>

//determiners
the :- N/N : (lambda $0:<e,t> $0)
is :- N/N : (lambda $0:<e,t> $0)
a :- N/N : (lambda $0:<e,t> $0)
of :- N/N : (lambda $0:<e,t> $0)
the :- NP/NP : (lambda $0:e $0)
is :- NP/NP : (lambda $0:e $0)
of :- NP/NP : (lambda $0:e $0)
the :- NP/N : (lambda $0:<e,t> (the:<<e,t>,e> $0))

that :- PP/(S\NP) : (lambda $0:<e,t> $0)
that :- PP/(S/NP) : (lambda $0:<e,t> $0)
which :- PP/(S\NP) : (lambda $0:<e,t> $0)
which :- PP/(S/NP) : (lambda $0:<e,t> $0)
are :- PP/PP : (lambda $0:<e,t> $0)

// for "bob 's office"; "the office of the chair"
s :- (NP/N)\NP : (lambda $0:e (lambda $1:<e,t> (the:<<e,t>,e> (lambda $2:e (and:<t*,t> (person:<pe,t> $0) ($1 $2) (possesses:<pe,<e,t>> $0 $2))))))
of :- (NP\N)/NP : (lambda $0:e (lambda $1:<e,t> (the:<<e,t>,e> (lambda $2:e (and:<t*,t> (person:<pe,t> $0) ($1 $2) (possesses:<pe,<e,t>> $0 $2))))))

// copula, etc.
are :- (N\N)/N : (lambda $0:<e,t> (lambda $1:<e,t> (lambda $2:e (and:<t*,t> ($0 $2) ($1 $2)))))
are :- (S\NP)/PP : (lambda $0:<e,t> $0)
does :- (S/NP)/(S/NP) : (lambda $0:<e,t> $0)
does :- (S\NP)/(S\NP) : (lambda $0:<e,t> $0)
is :- (S/NP)/(S/NP) : (lambda $0:<e,t> $0)
have :- (S/NP)/(S/NP) : (lambda $0:<e,t> $0)
is :- (S\NP)/(S\NP) : (lambda $0:<e,t> $0)
are there :- S\NP : (lambda $0:e true:t)
is :- (S\NP)/NP : (lambda $0:e (lambda $1:e (equals:<e,<e,t>> $1 $0)))

// negation
not :- N/N : (lambda $0:<e,t> (lambda $1:e (not:<t,t> ($0 $1))))
not :- PP/PP : (lambda $0:<e,t> (lambda $1:e (not:<t,t> ($0 $1))))
do not :- (S\NP)/(S\NP) : (lambda $0:<e,t> (lambda $1:e (not:<t,t> ($0 $1))))
no :- (S\NP)/(S\NP) : (lambda $0:<e,t> (lambda $1:e (not:<t,t> ($0 $1))))
excluding :- PP/NP : (lambda $0:e (lambda $1:e (not:<t,t> (equals:<e,<e,t>> $1 $0))))

// empty sentence modifier
tell me :- S/S : (lambda $0:<e,t> $0)
can you  :- S/S : (lambda $0:<e,t> $0)
please  :- S/S : (lambda $0:<e,t> $0)
please :- S\S : (lambda $0:e $0)
please :- S\S : (lambda $0:t $0)
is :- S/S : (lambda $0:t $0)

// quantifier
is :- (NP\N)/(NP/N)  : (lambda $0:<<e,t>,e> $0)
are  :- (NP\N)/(NP/N)  : (lambda $0:<<e,t>,e> $0)
with  :- (NP\N)/(NP/N)  : (lambda $0:<<e,t>,e> $0)

// np-list copy; including these entries here allows them to be used as candidates by the GENLEX procedure
i :- NP : me:pe
me :- NP : me:pe
you :- NP : self:self
yourself :- NP : self:self
peter stone :- NP : peter:pe
raymond mooney :- NP : ray:pe
dana ballard :- NP : dana:pe
kazunori iwata :- NP : kazunori:pe
matteo leonetti :- NP : matteo:pe
shiqi zhang :- NP : shiqi:pe
jivko sinapov :- NP : jivko:pe
stacy miller :- NP : stacy:pe
piyush khandelwal :- NP : piyush:pe
daniel urieli :- NP : daniel:pe
3402 :- NP : l3_402:ro
3404 :- NP : l3_404:ro
3416 :- NP : l3_416:ro
3418 :- NP : l3_418:ro
3420 :- NP : l3_420:ro
3422 :- NP : l3_422:ro
3430 :- NP : l3_430:ro
3432 :- NP : l3_432:ro
3436 :- NP : l3_436:ro
3428 :- NP : l3_428:ro
3426 :- NP : l3_426:ro
3414 :- NP : l3_414:ro
3414a :- NP : l3_414a:ro
3414b :- NP : l3_414b:ro
3412 :- NP : l3_412:ro
3502 :- NP : l3_502:ro
3508 :- NP : l3_508:ro
3510 :- NP : l3_510:ro
3512 :- NP : l3_512:ro
3516 :- NP : l3_516:ro
counter :- NP : coffeecounter:sc
coffee :- NP : coffee:it
hamburger :- NP : hamburger:it
cell phone :- NP : phone:it
trashcan :- NP : trashcan:it
calendar :- NP : calendar:it
