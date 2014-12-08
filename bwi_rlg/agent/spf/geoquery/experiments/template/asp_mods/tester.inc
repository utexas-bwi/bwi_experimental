## Tester
type=tester id=tester data=test parser=parser
## Filter to skip long sentences during learning
type=filter.sentence.length id=learningFilter length=%{trainingMaxSentenceLength}
