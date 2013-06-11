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



#include "upm/lib/lgeometry.hpp"

LSL_Point3D_container :: LSL_Point3D_container (void)
{
}

////-------------------------------------------------------------------------------------------------------------------

LSL_Point3D_container :: LSL_Point3D_container(unsigned int sz)
{
	pts = std::vector <LSL_Point3D_str> (sz);
}



////-------------------------------------------------------------------------------------------------------------------


LSL_Point3D_container :: ~LSL_Point3D_container ()
{
}
 

////-------------------------------------------------------------------------------------------------------------------

double distance_L2_XY (LSL_Point3D_str *pt0, LSL_Point3D_str *pt1)
{
	return ( sqrt( (pt0->x - pt1->x)*(pt0->x - pt1->x) + (pt0->y - pt1->y)*(pt0->y - pt1->y) ) );
}

////-------------------------------------------------------------------------------------------------------------------


double distance_L2_XY_sqr (LSL_Point3D_str *pt0, LSL_Point3D_str *pt1)
{
 	double dist = (pt0->x - pt1->x)*(pt0->x - pt1->x) + (pt0->y - pt1->y)*(pt0->y - pt1->y);
 	return(dist);
}
 
  
////-------------------------------------------------------------------------------------------------------------------


void order_bytheta_incart (std::vector <LSL_Point3D_str>& pts_out)
{

	std::vector <LSL_Point3D_polar_str> pts_polar_out;
		
	// convert and order
	conv2polar_func(pts_out, pts_polar_out);
	order_bytheta (pts_polar_out);

	// clear before pushing
	pts_out.clear();
	
	// convert in cartesian
	conv2cart_func(pts_polar_out, pts_out);	
	
	//free
	pts_polar_out.clear();
}
 
////-------------------------------------------------------------------------------------------------------------------


void order_bytheta (std::vector <LSL_Point3D_polar_str> &pts_polar_out)
{
	std::vector <LSL_Point3D_polar_str> pts_polar_tmp;
	LSL_Point3D_polar_str polar_p;
	
	std::vector <double> th_val;
	size_t *p = (size_t*) malloc(pts_polar_out.size() * sizeof(size_t));

	// sort index according to thetaval
	for(unsigned int i=0; i<pts_polar_out.size(); i++)
		th_val.push_back(pts_polar_out[i].theta);

        gsl_sort_index (p, &th_val[0], 1, pts_polar_out.size());	

     // order elements
     // ASCENDING order
     
       for(unsigned int i = 0; i < pts_polar_out.size(); i++)
       {
       
           polar_p.theta =   pts_polar_out[ p[i] ].theta;
           polar_p.alpha =   pts_polar_out[ p[i] ].alpha;
           polar_p.rho =   pts_polar_out[ p[i] ].rho;
           polar_p.ch =   pts_polar_out[ p[i] ].ch;
		   polar_p.status =   pts_polar_out[ p[i] ].status;
		   polar_p.w =   pts_polar_out[ p[i] ].w;
		   polar_p.label =   pts_polar_out[ p[i] ].label;
		   polar_p.id =   pts_polar_out[ p[i] ].id;
		   polar_p.tag =   pts_polar_out[ p[i] ].tag;
           pts_polar_tmp.push_back(polar_p);
           // printf("%f  ",polar_p.theta );
       }
             
	pts_polar_out.clear();	
	pts_polar_out = pts_polar_tmp;
	pts_polar_tmp.clear();
	
	free(p);
}
 

////-------------------------------------------------------------------------------------------------------------------

LSL_Point3D_container :: LSL_Point3D_container(std::vector <LSL_Point3D_str> &ptvec)
{
	pts = ptvec;
}

////-------------------------------------------------------------------------------------------------------------------


void LSL_Point3D_container :: compute_cog(LSL_Point3D_str *pts_out)
{
	double x_sum = 0;
	double y_sum = 0;
	double z_sum = 0;
		
	// centroid
	for(unsigned int i=0 ; i < pts.size(); i++)
	{
		x_sum += pts[i].x;
		y_sum += pts[i].y;
		z_sum += pts[i].z;
	}		

	pts_out -> x = x_sum/(double)pts.size();	
	pts_out  ->y = y_sum/(double)pts.size();
	pts_out  ->z = z_sum/(double)pts.size();
}



////-------------------------------------------------------------------------------------------------------------------

/**
 * It had a problem because it was initializing the size of the vector.
 * Effectively the result was double the expected size and the first NO_SIZE elements where empty.
 * The rest that was push_back were never accessed. Removed the initialization and fixed it.
 * @param pts_coord
 * @param coord_sel
 */
void LSL_Point3D_container :: get_coords(std::vector <double> &pts_coord, char coord_sel)
{
 
	//pts_coord = std::vector <double> (pts.size());
	if(coord_sel == GEOMETRY_COORD_X)	
	{
		// centroid
		for(unsigned int i=0 ; i < pts.size(); i++)
			pts_coord.push_back(pts[i].x);
	}

	if(coord_sel == GEOMETRY_COORD_Y)	
	{
		// centroid
		for(unsigned int i=0 ; i < pts.size(); i++)
			pts_coord.push_back(pts[i].y);
	}

	if(coord_sel == GEOMETRY_COORD_Z)	
	{
		// centroid
		for(unsigned int i=0 ; i < pts.size(); i++)
			pts_coord.push_back(pts[i].z);
	}
	
}
  

////-------------------------------------------------------------------------------------------------------------------


void conv2polar_func(std::vector <LSL_Point3D_str>& pts_in, std::vector <LSL_Point3D_polar_str>& pts_polar_out)
{
	// theta is angle plane x-y	
	// alpha is angle plane z-y	

	LSL_Point3D_str origin;	
	LSL_Point3D_polar_str polar_pt;
	
	origin.x = 0;
	origin.y = 0;
	origin.z = 0;
	
	for(unsigned int i= 0; i < pts_in.size() ; i++)
	{
		// cart to pol conver
		polar_pt.rho = distance_L2_XY (&pts_in[i], &origin);
		polar_pt.theta =  atan2(pts_in[i].y, pts_in[i].x);
		polar_pt.alpha =  atan2(sqrt(pts_in[i].x*pts_in[i].x + pts_in[i].y * pts_in[i].y) , pts_in[i].z);
	    polar_pt.ch =  pts_in[i].ch;
	    polar_pt.status =  pts_in[i].status;
 		polar_pt.w =   pts_in[ i].w;
 		polar_pt.tag =   pts_in[ i].tag; 
		polar_pt.label =   pts_in[ i ].label;
		polar_pt.id    =   pts_in[ i ].id;
		   	    	
		// pump it in!
		pts_polar_out.push_back(polar_pt);
	}

}


////-------------------------------------------------------------------------------------------------------------------


void conv2cart_func(std::vector <LSL_Point3D_polar_str>& pts_polar_in, std::vector <LSL_Point3D_str>& pts_out)
{
	// theta is angle plane x-y	
	// alpha is angle plane z-y	

	LSL_Point3D_str pt;	
	
	for(unsigned int i= 0; i < pts_polar_in.size() ; i++)
	{
		// pol2cart conver
		pt.x = pts_polar_in[i].rho * sin(pts_polar_in[i].alpha) * cos(pts_polar_in[i].theta);
		pt.y = pts_polar_in[i].rho * sin(pts_polar_in[i].alpha) * sin(pts_polar_in[i].theta);
		pt.z = pts_polar_in[i].rho * cos(pts_polar_in[i].alpha);
		pt.ch =  pts_polar_in[i].ch;
		pt.status =  pts_polar_in[i].status;
		pt.w =  pts_polar_in[i].w;
		pt.label =  pts_polar_in[i].label;
		pt.id =  pts_polar_in[i].id;
		pt.tag =  pts_polar_in[i].tag;
		
//		printf("*[%f %f %f] %d\n",pt.x ,pt.y, pt.z, pt.ch );
						
		// pump it in!
		pts_out.push_back(pt);
	}

}


////-------------------------------------------------------------------------------------------------------------------



void  get_circle_param(LSL_Point3D_container *laserfeat_cluster, 	LSL_Point3D_str *circle_param)
{
	// takes only plane XY

	std::vector <double> A (laserfeat_cluster->pts.size()*3);
	std::vector <double> B (laserfeat_cluster->pts.size());

	// fill A
	for(unsigned int i=0,a=0; i < laserfeat_cluster->pts.size(); i++)
	{
		A[a] = -2.0 * laserfeat_cluster->pts[i].x;
		a = a+1;
		A[a] = -2.0 * laserfeat_cluster->pts[i].y;
		a = a+1;
		A[a]= 1.0;						
		a = a+1;
	}

	// fill B
	for(unsigned int i=0; i < laserfeat_cluster->pts.size(); i++)
		B[i] = -laserfeat_cluster->pts[i].x*laserfeat_cluster->pts[i].x - laserfeat_cluster->pts[i].y*laserfeat_cluster->pts[i].y;
	     
        gsl_matrix_view m    = gsl_matrix_view_array (&A[0], laserfeat_cluster->pts.size(), 3);     
       gsl_vector_view b      = gsl_vector_view_array (&B[0], laserfeat_cluster->pts.size());
       
	// GSL SOLVER
       gsl_vector *x = gsl_vector_alloc (3);
       gsl_vector *tau= gsl_vector_alloc (3);
       gsl_vector *residual = gsl_vector_alloc (laserfeat_cluster->pts.size());
              
       gsl_linalg_QR_decomp (&m.matrix, tau);       
       gsl_linalg_QR_lssolve (&m.matrix, tau, &b.vector,  x,  residual);

      
       // in Z I put rc
       circle_param -> x = x->data[0];
       circle_param -> y = x->data[1];       
       circle_param -> z = sqrt( x->data[0] * x->data[0] + x->data[1] * x->data[1] - x->data[2]);

       //printf ("%f %f %f\n",xc,yc,rc);
	                     
       gsl_vector_free (x);
       gsl_vector_free (tau);
       gsl_vector_free (residual);              
}



////-------------------------------------------------------------------------------------------------------------------



void get_line_param(LSL_Point3D_container *laserfeat_cluster, 	LSL_Point3D_str *circle_param)
{
	// takes only plane XY
	
	std::vector <double> A;
	std::vector <double> B;
	
	// fill A
	for(unsigned int i=0; i < laserfeat_cluster->pts.size(); i++)
	{
		A.push_back( laserfeat_cluster->pts[i].x);
		A.push_back(1.0);						
	}

	// fill B
	for(unsigned int i=0; i < laserfeat_cluster->pts.size(); i++)
		B.push_back( laserfeat_cluster->pts[i].y);

	     
       gsl_matrix_view m    = gsl_matrix_view_array (&A[0], laserfeat_cluster->pts.size(), 2);     
       gsl_vector_view b      = gsl_vector_view_array (&B[0], laserfeat_cluster->pts.size());
       
	// GSL SOLVER
       gsl_vector *x = gsl_vector_alloc (2);
       gsl_vector *tau= gsl_vector_alloc (2);
       gsl_vector *residual = gsl_vector_alloc (laserfeat_cluster->pts.size());
              
       gsl_linalg_QR_decomp (&m.matrix, tau);       
       gsl_linalg_QR_lssolve (&m.matrix, tau, &b.vector,  x,  residual);

     
       // in Z I put rc
       circle_param -> x = x->data[0];
       circle_param -> y = x->data[1];       
       circle_param -> z = 0;

       //printf ("%f %f %f\n",xc,yc,rc);
	                     
       gsl_vector_free (x);
       gsl_vector_free (tau);
       gsl_vector_free (residual);              
}

////-------------------------------------------------------------------------------------------------------------------
 
