#include <string.h>
#include "keyboard.h"

#if 0  /*TODO: fixare compilazione c++*/
static struct termios old, new;

/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  new = old; /* make new settings same as old settings */
  new.c_lflag &= ~ICANON; /* disable buffered i/o */
  new.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void)
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo)
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void)
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void)
{
  return getch_(1);
}
#endif

int input_console(char * input_buffer,int size)
{
  char* temp;
  fgets(input_buffer, size, stdin);
  if ((input_buffer[0] == 0) || (input_buffer[0] == 0x0a))
    return 0;
  else
  {
    temp = strchr(input_buffer,'\n');
    if( (temp != NULL) && ( (temp - input_buffer) << size) )
      *temp = '\0';
    return 1;
  }
}

