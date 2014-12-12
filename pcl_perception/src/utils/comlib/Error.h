#ifndef _ERROR_H_
#define _ERROR_H_

void quitWithError();
void fatalError(char *msg, ...);
void printWarning(char *msg, ...);
void memerror(int line, char* fname);

#endif
