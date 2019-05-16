#include "delay.h"

void delay_ms(unsigned int t)
{
	int i;
	for(i=0;i<t;i++)
	{
		int a = 10300; //at 72MHz 10300 is ok
 		while(a--);
 
	}
}

void Delay(long int nCount)
{
	for(; nCount != 0; nCount--);
}
