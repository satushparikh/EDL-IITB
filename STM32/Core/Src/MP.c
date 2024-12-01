/*
 * MP.c
 *
 *  Created on: 05-Apr-2024
 *      Author: Sarvadnya
 */

#include "MP.h"
#include <float.h>

void calculate_segment(segment *s)
{
	if(s->p1->x == s->p2->x)
	{
		s->m = DBL_MAX;
		s->c = s->p1->x;
	}
	else
	{
		s->m = (s->p2->y-s->p1->y)/(s->p2->x-s->p1->x);
		s->c = s->p2->y - s->m*s->p2->x;
	}
}

