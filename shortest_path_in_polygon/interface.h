#ifndef __interface_h
#define __interface_h

#define TRUE 1
#define FALSE 0

int triangulate_polygon(int ncontours,
	int cntr[],
	double(*vertices)[2],
	int(*triangles)[3]);
int is_point_inside_polygon(double vertex[2]);

#endif /* __interface_h */
