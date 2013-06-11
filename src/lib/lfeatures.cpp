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



#include "upm/lib/lfeatures.hpp"


//~ COMMENT: the scan line must be ordered by angle
//~ COMMENT: at least 3 points are needed to compute the featureset

void LSL_lfeatures_class ::descr(void)
{
 	feature_descr.push_back("feature 01: Num of points");
 	feature_descr.push_back("feature 02: std from centroid (in 2D x and y)"); 	
	feature_descr.push_back("feature 03: Mean average deviation from median (in 2D xy)");
	feature_descr.push_back("feature 04: width");
	feature_descr.push_back("feature 05: circularity  (in 2D plane xy)");
	feature_descr.push_back("feature 06: radius fit circle  (in 2D plane xy)");
	feature_descr.push_back("feature 07: boundary length (in 2D plane xy) (open contour)");
	feature_descr.push_back("feature 08: boundary regularity (in 2D plane xy)");
	feature_descr.push_back("feature 09: mean curvature  (in 2D plane xy)");
	feature_descr.push_back("feature 10: mean angular difference  (in 2D plane xy)");
	feature_descr.push_back("feature 11: aspect ratio1: GEOMETRY_MAX(std_x,std_y) / GEOMETRY_MAX(std_x,std_y)");
	feature_descr.push_back("feature 12: kurtosis (in 2D plane xy)");
	feature_descr.push_back("feature 13: linearity");
	feature_descr.push_back("feature 14: circularity  (in 2D plane xy) normalized");
	feature_descr.push_back("feature 15: linearity  (in 2D plane xy) normalized");	
	feature_descr.push_back("feature 16: distance");		
	feature_descr.push_back("feature 01_comp: smallest segment-segment distance ");			
}


////-------------------------------------------------------------------------------------------------------------------

LSL_lfeatures_class::LSL_lfeatures_class(const std::vector <int> &featnum, const std::vector <int> &featnum_comparative)
{
	//~ standard feats
	for (unsigned int i = 0 ; i< featnum.size(); i++)
	{
		if(featnum[i] == 0)
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_01);
		if(featnum[i] == 1)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_02);		
		if(featnum[i] == 2)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_03);		
		if(featnum[i] == 3)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_04);		
		if(featnum[i] == 4)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_05);		
		if(featnum[i] == 5)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_06);		
		if(featnum[i] == 6)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_07);		
		if(featnum[i] == 7)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_08);		
		if(featnum[i] == 8)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_09);		
		if(featnum[i] == 9)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_10);		
		if(featnum[i] == 10)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_11);		
		if(featnum[i] == 11)
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_12);
		if(featnum[i] == 12)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_13);		
		if(featnum[i] == 13)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_14);		
		if(featnum[i] == 14)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_15);		
		if(featnum[i] == 15)			
			ptr_ezfeature_vec_1D.push_back (&LSL_lfeatures_class::feature_16);		
	}
	
	//~ feats of relations
	for (unsigned int i = 0 ; i< featnum_comparative.size(); i++)
	{
		if(featnum[i] == 0)
			ptr_ezfeature_vec_1D_cmp.push_back (&LSL_lfeatures_class::feature_01_comparative);
	}
	
}

////-------------------------------------------------------------------------------------------------------------------
 
void LSL_lfeatures_class ::  compute_descriptor(std::vector <LSL_Point3D_container> &all_laserfeat_cluster, std::vector < std::vector <Real> > &descriptor)
{		

	descriptor = std::vector < std::vector <Real> > (all_laserfeat_cluster.size(), (std::vector <Real> ( ptr_ezfeature_vec_1D.size() +ptr_ezfeature_vec_1D_cmp.size() ) ) );
	
	//~ all segments
	for (unsigned int c = 0 ; c< all_laserfeat_cluster.size(); c++)
	{
		unsigned int fidx = 0;
		//~ do standard feats
		for (unsigned int f = 0; f < ptr_ezfeature_vec_1D.size(); f++)		
		{
			Real value;
			char ret = CALL_MEMBER_FN(*this, ptr_ezfeature_vec_1D[f]) (&all_laserfeat_cluster[c], &value); 
				
			if(!ret)
			{
				printf("lfeatures.cpp: Error in feature computation SMP.   Check data validity [f: %d c: %d]\n",f,c);
				exit(1);
			}
				
			descriptor[c][fidx] = value;
			fidx++;
		}
			
		//~  do rel feats
		for (unsigned int f = 0; f < ptr_ezfeature_vec_1D_cmp.size(); f++)		
		{
			Real value;
			char ret = CALL_MEMBER_FN_CMP(*this, ptr_ezfeature_vec_1D_cmp[f]) (all_laserfeat_cluster, c, &value); 
				
			if(!ret)
			{
				printf("lfeatures.cpp: Error in feature computation CMP. Check data validity [f: %d c: %d]\n",f,c);
				exit(1);
			}
				
			descriptor[c][fidx] = value;
			fidx++;
		}			
	}
}


////-------------------------------------------------------------------------------------------------------------------

/// feature 01: Num of points
int LSL_lfeatures_class ::  feature_01(LSL_Point3D_container *laserfeat_cluster, Real *out)
{		
	*out = laserfeat_cluster -> pts.size();
	return(1);
}
 

////-------------------------------------------------------------------------------------------------------------------
/// feature 02: std from centroid (in 2D x and y)
int LSL_lfeatures_class ::  feature_02(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	LSL_Point3D_str pts_cog;	
	char ret = 1;
	
	// cog	
	laserfeat_cluster -> compute_cog(&pts_cog);

	// compute vectorial std from 
	double s_sum = 0;
	
	for(unsigned int i = 0; i < laserfeat_cluster -> pts.size(); i++ )
	{
		double val = distance_L2_XY (&pts_cog, &laserfeat_cluster -> pts[i]);
		s_sum += val*val;
	}
	
	if(s_sum == 0)
		ret = 0;
	
	*out  = sqrt( (1.0 /  ((double)laserfeat_cluster -> pts.size()))   * s_sum);
	return(ret);
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 03: Mean average deviation from median (in 2D xy)
int LSL_lfeatures_class ::  feature_03(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;
	double s_sum = 0;
	LSL_Point3D_str pts_median;
	
	std::vector <double> pts_coordx,pts_coordy;
	
	// take x
	laserfeat_cluster -> get_coords(pts_coordx, GEOMETRY_COORD_X);		
	// sort data
    gsl_sort (&pts_coordx[0], 1, pts_coordx.size());
	// median
    pts_median.x = gsl_stats_median_from_sorted_data (&pts_coordx[0], 1, pts_coordx.size());	
	// take y
	laserfeat_cluster -> get_coords(pts_coordy, GEOMETRY_COORD_Y);			
	// sort data
     gsl_sort (&pts_coordy[0], 1, pts_coordy.size());
	// median
     pts_median.y = gsl_stats_median_from_sorted_data (&pts_coordy[0], 1, pts_coordy.size());
	// project on xy plane
	pts_median.z = 0;

	for(unsigned int i = 0; i < laserfeat_cluster -> pts.size(); i++ )
	{
		double val = distance_L2_XY (&pts_median, &laserfeat_cluster -> pts[i]);
		s_sum += val * val;	
	}

	
	if(s_sum == 0)
		ret = 0;

        //printf("feature 03: median x %f y %f , sum %f \n",pts_median.x, pts_median.y, s_sum);

	*out = sqrt( ( 1.0 /  (double)laserfeat_cluster -> pts.size() )   * s_sum);
	
	return(ret);
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 04: width
int LSL_lfeatures_class ::  feature_04(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;
	
	//~ endpoints
	int lastidx = laserfeat_cluster->pts.size()-1;
	
	//~ 2 pts are enough
	if(laserfeat_cluster->pts.size() > 1)
		*out = distance_L2_XY (&laserfeat_cluster->pts[0] , &laserfeat_cluster->pts[lastidx]);
	else
		ret = 0;

	return(ret);	
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 05: circularity (in 2D plane xy)
int LSL_lfeatures_class ::  feature_05(LSL_Point3D_container *laserfeat_cluster, Real *out)
{	 
	char ret = 1;
	
	LSL_Point3D_str circle_param;
	double s_sum =0;
		   
	//~ 3 pts to fit a circle
	if(laserfeat_cluster-> pts.size() > 2)
	{
		// get cirlce parameters xc yc rc (rc is stored in Z var)
		get_circle_param( laserfeat_cluster, &circle_param);

		double xc = circle_param.x;
		double yc = circle_param.y;
		double rc = circle_param.z;

		//~ avoid nans and aligned points
		if(isnan(rc)  || rc >= L_HUGENUM)
		{
			s_sum = L_HUGENUM;
		}
		else
		{
			// residual sum
			for(unsigned int i=0; i < laserfeat_cluster->pts.size(); i++)
			{
				double dx = xc - laserfeat_cluster->pts[i].x;
				double dy = yc - laserfeat_cluster->pts[i].y;
				double residual = rc - sqrt(dx * dx + dy * dy);
				s_sum += residual*residual;
			}	
		}
		ret = 1;
	}
	else
		ret = 0;

	*out = s_sum;
	
	return(ret);
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 06: radius fit circle  (in 2D plane xy)
int LSL_lfeatures_class ::  feature_06(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;		
	LSL_Point3D_str circle_param;

	//~ 3 pts to fit a circle
	if(laserfeat_cluster-> pts.size() > 2)
	{
		// get cirlce parameters xc yc rc (rc is stored in Z var)
		get_circle_param( laserfeat_cluster, &circle_param);
		if(isnan(circle_param.z)  || circle_param.z >= L_HUGENUM)
			circle_param.z = L_HUGENUM;

		*out = circle_param.z;		
	}
	else
		ret = 0;

	return(ret);
}


////-------------------------------------------------------------------------------------------------------------------


/// feature 07: boundary length (in 2D plane xy)
int LSL_lfeatures_class ::  feature_07(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	double s_sum = 0;
	char ret = 1;	

	if(laserfeat_cluster->pts.size() > 1)
	{
		for(unsigned int i = 0; i < laserfeat_cluster->pts.size() - 1; i++)
		{
			//~ disregard z
			laserfeat_cluster->pts[i].z = 0;
			laserfeat_cluster->pts[i+1].z = 0;
 			s_sum += distance_L2_XY (&laserfeat_cluster->pts[i] , &laserfeat_cluster->pts[i+1]);
		}
	}
	else
		ret = 0;

    *out =s_sum;
	return(ret);
}


////-------------------------------------------------------------------------------------------------------------------


/// feature 08:   boundary regularity (in 2D plane xy)
int LSL_lfeatures_class ::  feature_08(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	char ret=1;

	if(laserfeat_cluster->pts.size() > 2)
	{
		std::vector <double> dist ( laserfeat_cluster->pts.size() - 1);
		
		int c = 0;
		for(unsigned int i = 0; i <laserfeat_cluster->pts.size() - 1; i++,c++)
			dist[c] = distance_L2_XY (&laserfeat_cluster->pts[i] , &laserfeat_cluster->pts[i + 1]);


		*out = gsl_stats_sd (&dist[0], 1, dist.size());
	}
	else	
		ret = 0;

	return(ret);
}


////-------------------------------------------------------------------------------------------------------------------

/// feature 09: mean curvature  (in 2D plane xy)
int LSL_lfeatures_class ::  feature_09(LSL_Point3D_container *laserfeat_cluster, Real *out)
{	 
	//~ the result is the inverse of the fitted circle radius
	char ret = 1;
	double k_sum = 0;
	double area;

	if(laserfeat_cluster->pts.size() > 2)
	{
		std::vector <double> d (3);
		for(unsigned int i = 1; i <laserfeat_cluster->pts.size() -1; i++)
		{
				d[0] =  distance_L2_XY (&laserfeat_cluster->pts[i - 1], &laserfeat_cluster->pts[i]);
       		 	d[1] =  distance_L2_XY (&laserfeat_cluster->pts[i], &laserfeat_cluster->pts[i+1]);
       		 	d[2] =  distance_L2_XY (&laserfeat_cluster->pts[i - 1], &laserfeat_cluster->pts[i + 1]);
				std::sort(d.begin(), d.end());

				//~ numerically stable Hero's formula
      			area = (d[0] + d[1] + d[2]) * (d[2] - (d[0] - d[1])) * (d[2] + (d[0] - d[1])) * (d[0] + (d[1] - d[2])) + L_EPSNUM;

       			//~ no curvature
       			if(area <0)
					area = 0;
       			area = 0.25 * sqrt(area);
       			double denom = d[0]*d[1]*d[2] + L_EPSNUM;
       			k_sum += (4.0 * area) / denom;
       			
    	}
 		*out =  k_sum / ((double)laserfeat_cluster->pts.size());
	}
	else
		ret = 0;

	return(ret);
}


////-------------------------------------------------------------------------------------------------------------------

/// feature 10: mean angular difference on convex hull border (in 2D plane xy)
int LSL_lfeatures_class ::  feature_10(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;
	double res;
	double sc_prd, n0, n1, div;
	double ang_sum = 0;
	LSL_Point3D_str pt0, pt1;
	LSL_Point3D_str origin;	
	
	origin.x = 0;
	origin.y = 0;
	origin.z = 0;
				
 	if(laserfeat_cluster->pts.size() > 2)
	{
		for(unsigned int i = 1; i <laserfeat_cluster->pts.size()-1; i++)
		{
 			 // the CW or CCW sense eis not important since  they're consecutive
			
		       	 pt0.x = laserfeat_cluster->pts[i].x - laserfeat_cluster->pts[i-1].x;
	       		 pt0.y = laserfeat_cluster->pts[i].y - laserfeat_cluster->pts[i-1].y;
	       		 
	       		 pt1.x = laserfeat_cluster->pts[i + 1].x - laserfeat_cluster->pts[i].x;
	       		 pt1.y = laserfeat_cluster->pts[i + 1].y - laserfeat_cluster->pts[i].y;
	       		 
	     		 n0 =  distance_L2_XY(&pt0, &origin);
	     		 n1 =  distance_L2_XY(&pt1, &origin);
	     		 
	     		 div = n0 * n1;
	     		
	     		// very small curvature if points collide: do not sum ang 
	     		 if(div > 0)
	     		 {
		       		 // scalar product
	       			 sc_prd = pt0.x * pt1.x + pt0.y * pt1.y;
					 res = sc_prd / div;
					
					 //~ numerical instabilities
					 if( res > 0.0 )
					 	res-=L_EPSNUM;

					 if(res < 0.0)
					 	res+=L_EPSNUM;
					 
					 if(fabs(res) > 1.0)
					 {
					 	printf("ERROR: memory corruption in feature10\n") ;
					 	exit(1);
					 }	

					 ang_sum += acos(  res );
	       		  }
		}
		
		*out = ang_sum / (double)(laserfeat_cluster->pts.size() - 2);
    }   	
    else
		ret = 0;

	return(ret);
	
}

////-------------------------------------------------------------------------------------------------------------------

/** feature 11:  aspect ratio1: GEOMETRY_MAX(std_x,std_y) / GEOMETRY_MIN(std_x,std_y)
    This initially had a problem due to the call to get_coords(pts, coord) . Fixed.
    @see LSL_Point3D_container::get_coords (std::vector< double > &pts_coord, char coord_sel)
*/
int LSL_lfeatures_class ::  feature_11(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;
	
	double GEOMETRY_MIN_v, GEOMETRY_MAX_v, std1, std2;
	std::vector <double> pts_coordx,pts_coordy;
	
	laserfeat_cluster -> get_coords(pts_coordx, GEOMETRY_COORD_X);
        //printf("feature 11 size: cluster %d pts_coord %d\n",  laserfeat_cluster->pts.size() , pts_coord.size());

	std1 = gsl_stats_sd (&pts_coordx[0], 1, laserfeat_cluster->pts.size() -1) ;
        //printf("std1 %f\n", std1);

	laserfeat_cluster -> get_coords(pts_coordy, GEOMETRY_COORD_Y);
	std2 = gsl_stats_sd (&pts_coordy[0], 1, laserfeat_cluster->pts.size() -1) ;
        //printf("std2 %f\n", std2);


	GEOMETRY_MAX_v = GEOMETRY_MAX(std1, std2);	
	GEOMETRY_MIN_v = GEOMETRY_MIN(std1, std2);

	*out = (1.0 + GEOMETRY_MIN_v) / ( 1.0 + GEOMETRY_MAX_v) ;
	//printf("feature 11: %f\n",*out);
	return(ret);
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 12:  kurtosis (2D: plane xy)
int LSL_lfeatures_class ::  feature_12(LSL_Point3D_container *laserfeat_cluster, Real *out)
{

	LSL_Point3D_str pts_cog;
	double s_sum = 0;

	//~ at least 2pt
	if(	laserfeat_cluster -> pts.size() < 2)
		return(0);
			
	// get COG
	laserfeat_cluster->compute_cog(&pts_cog);

	// compute kurtosis
	for(unsigned int i = 0 ; i < laserfeat_cluster -> pts.size(); i++)
	{
		double val = distance_L2_XY (&laserfeat_cluster -> pts[i], &pts_cog);
		s_sum += val*val*val*val;
	}
	
	// compute 2D std
	Real out1;
	feature_02(laserfeat_cluster,  &out1);
	
	double num = s_sum / (double)laserfeat_cluster -> pts.size();
	double denom = out1 * out1 + L_EPSNUM;
	*out =  num / denom;

	return(1);
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 13: linearity
int LSL_lfeatures_class ::  feature_13(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;	
	double s_sum = 0;
		
	if(laserfeat_cluster-> pts.size() > 1)
	{
		LSL_Point3D_str line_param, pts_cog;

		// get line parameters 
		get_line_param( laserfeat_cluster, &line_param);

		double m = line_param.x;
		double q = line_param.y;
		
		//~ vertical line
		if(fabs(m) > L_HUGENUM)
		{
			laserfeat_cluster -> compute_cog(&pts_cog);

			// residual sum
			for(unsigned int i=0; i < laserfeat_cluster->pts.size(); i++)
			{
				double dx = laserfeat_cluster->pts[i].x - pts_cog.x;
				s_sum += dx * dx;
			}	
		}
		else
		{
			// residual sum
			for(unsigned int i=0; i < laserfeat_cluster->pts.size(); i++)
			{
				double rr = (m * laserfeat_cluster->pts[i].x + q  - laserfeat_cluster->pts[i].y);
				s_sum += rr*rr;
			}
		}		
	}
	else
		ret = 0;

	//~ printf("%f\n",s_sum);		
	*out =  s_sum;

	return(ret);
}


////-------------------------------------------------------------------------------------------------------------------


/// feature 14: circularity normed (in 2D plane xy)
int LSL_lfeatures_class ::  feature_14(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	Real out1;
	char ret = feature_05(laserfeat_cluster, &out1);
	*out = out1 / (double)laserfeat_cluster-> pts.size();
	return(ret);
}




////-------------------------------------------------------------------------------------------------------------------


/// feature 15: linearity normed (in 2D plane xy)
int LSL_lfeatures_class ::  feature_15(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	Real out1;
	char ret = feature_13(laserfeat_cluster, &out1);
	*out = out1 / (double)laserfeat_cluster-> pts.size();
	return(ret);
}



////-------------------------------------------------------------------------------------------------------------------

/// feature 16: distance
int LSL_lfeatures_class ::  feature_16(LSL_Point3D_container *laserfeat_cluster, Real *out)
{
	// origin
	LSL_Point3D_str origin, pts_cog;	
	
	origin.x = 0;
	origin.y = 0;
	origin.z = 0;

	// get COG
	laserfeat_cluster->compute_cog(&pts_cog);

	// compute distance
	double dist = distance_L2_XY(&pts_cog, &origin);
	
	*out = dist;
	return(1);
}


////-------------------------------------------------------------------------------------------------------------------


/// distance nearest cluster (via cogs)
int LSL_lfeatures_class ::  feature_01_comparative(std::vector <LSL_Point3D_container> &all_laserfeat_cluster, unsigned int curidx, Real *out)
{		
	float bestdist = FLT_MAX;
	LSL_Point3D_str pts_cog;	

	all_laserfeat_cluster[curidx].compute_cog(&pts_cog);
	for (unsigned int i = 0 ; i< all_laserfeat_cluster.size(); i++)
	{
		if(i != curidx)
		{
			LSL_Point3D_str pts_curcog;	
			all_laserfeat_cluster[i].compute_cog(&pts_curcog);
			double curdist = distance_L2_XY(&pts_cog, &pts_curcog);
			
			if(curdist < bestdist)
				bestdist = curdist;
		}
	}
	*out = bestdist;
	return(1);
}
 
