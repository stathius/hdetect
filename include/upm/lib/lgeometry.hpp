/*****************************************************************
 *
 * This file is part of the People2D project
 *
 * People2D Copyright (c) 2011 Luciano Spinello
 *
 * This software is licensed under the "Creative Commons 
 * License (Attribution-NonCommercial-ShareAlike 3.0)" 
 * and is copyrighted by Luciano Spinello
 * 
 * Further information on this license can be found at:
 * http://creativecommons.org/licenses/by-nc-sa/3.0/
 * 
 * People2D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 *
 *****************************************************************/



#ifndef LIBGEOMETRY_H
#define LIBGEOMETRY_H

// C++ specific
#include <iostream>
#include <vector>
#include <list>

// C specific
#include <math.h>
 
// GSL lib
#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_sort_double.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>

// defines
 #define GEOMETRY_COORD_X 0
#define GEOMETRY_COORD_Y 1
#define GEOMETRY_COORD_Z 2

#define GEOMETRY_HUGENUMBER 2000000
#define GEOMETRY_DWARFNUMBER 1e-6

#define GEOMETRY_MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define GEOMETRY_MIN(a, b)  (((a) < (b)) ? (a) : (b))

typedef struct
{
	double   x;
	double   y;
	double   z;
	int 	 ch;
	int 	 status;
	double 	 w;
	double 	tag;
	int 	 label;
	int 	 id;
}LSL_Point3D_str;

typedef struct
{
	double  theta;
	double  alpha;
	double 	rho;
	int 	ch;
	int 	status;
	double 	 w;
	int 	 label;	
	int 	 id;
	double 	tag;
}LSL_Point3D_polar_str;

class LSL_Point3D_container 
{
	private:

		
	public: 
		std::vector <LSL_Point3D_str> pts;
		
		// constructor
		LSL_Point3D_container();	
		LSL_Point3D_container(unsigned int sz);	
		LSL_Point3D_container(std::vector <LSL_Point3D_str> &ptvec);	

		// compute cog
		void compute_cog(LSL_Point3D_str *pts_out);	

		// get x y or z
		void get_coords(std::vector <double> &pts_coord, char coord_sel);
		
		// compute cog
		void conv2polar(std::vector <LSL_Point3D_polar_str> &pts_polar_out);

		// destructor
		~LSL_Point3D_container();		
};

 

// p2p 
double distance_L2_XY (LSL_Point3D_str *pt0, LSL_Point3D_str *pt1);
double distance_L2_XY_sqr (LSL_Point3D_str *pt0, LSL_Point3D_str *pt1);

// convertstuf
void conv2polar_func(std::vector <LSL_Point3D_str>& pts_in, std::vector <LSL_Point3D_polar_str>& pts_polar_out);
void conv2cart_func(std::vector <LSL_Point3D_polar_str>& pts_polar_in, std::vector <LSL_Point3D_str>& pts_out);

// order points
void order_bytheta (std::vector <LSL_Point3D_polar_str> &pts_polar_out);
void order_bytheta_incart (std::vector <LSL_Point3D_str>& pts_out);

//~ line param and circle param from a set of pts
void get_line_param(LSL_Point3D_container *laserfeat_cluster, 	LSL_Point3D_str *line_param);
void get_circle_param(LSL_Point3D_container *laserfeat_cluster, 	LSL_Point3D_str *circle_param);


#endif
