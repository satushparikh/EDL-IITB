/*
 * MP.h
 *
 *  Created on: 05-Apr-2024
 *      Author: Sarvadnya
 */

#ifndef SRC_MP_H_
#define SRC_MP_H_

#include "stdint.h"
#include "main.h"
#include <float.h>

typedef struct{
	double x;
	double y;
}point;

typedef struct{
	point *p1;
	point *p2;
	double m;
	double c;
}segment;

void calculate_segment(segment *s);

#endif /* SRC_MP_H_ */
