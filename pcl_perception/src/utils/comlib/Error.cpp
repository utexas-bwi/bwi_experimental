#include <stdio.h>
#include <stdlib.h> 
#include <stdarg.h>


void quitGracefully()
{
  exit (0);
}

void quitWithError()
{
  exit (1);
}

void printMessage (char *msg1, char *msg2, va_list ap)
{
  fflush (stderr);
  fflush (stdout);
  fprintf (stderr,"\n%s: ",msg1);
  vfprintf (stderr,msg2,ap);
  fprintf (stderr,"\n");
  fflush (stderr);
}


void fatalError(char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  printMessage ("Fatal Error",msg,ap);
  quitWithError();
}

void printWarning(char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  printMessage ("WARNING",msg,ap);
}

void vaPrint (char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  vprintf (msg,ap);
}


void memerror(int line, char* fname)
{
  fprintf(stderr, "Out of memory error ar line:%d in File:%s\n", line, fname);
  quitWithError();
}

