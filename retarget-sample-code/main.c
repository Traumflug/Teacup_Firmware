
#include <serial_api.h>

#include <stdio.h>

extern serial_t serial_line;

void retarget_init(void);


int main() {

  retarget_init();

  printf("Hello, world!\n");

  for ( ; ; );
}
