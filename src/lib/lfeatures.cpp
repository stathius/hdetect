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


#include "hdetect/lib/lfeatures.hpp"

//using namespace std;
using std::vector;

//~ COMMENT: the scan line must be ordered by angle
//~ COMMENT: at least 3 points are needed to compute the featureset

void lfeatures_class::descr(void)
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

lfeatures_class::lfeatures_class(int feature_set)
{
	this->feature_set = feature_set;

	//~ standard feats
	if (feature_set == 0)
	{
		ptr_feature_vector.push_back (&lfeatures_class::feature_01);
		ptr_feature_vector.push_back (&lfeatures_class::feature_02);
		ptr_feature_vector.push_back (&lfeatures_class::feature_03);
		ptr_feature_vector.push_back (&lfeatures_class::feature_04);
		ptr_feature_vector.push_back (&lfeatures_class::feature_05);
		ptr_feature_vector.push_back (&lfeatures_class::feature_06);
		ptr_feature_vector.push_back (&lfeatures_class::feature_07);
		ptr_feature_vector.push_back (&lfeatures_class::feature_08);
		ptr_feature_vector.push_back (&lfeatures_class::feature_09);
		ptr_feature_vector.push_back (&lfeatures_class::feature_10);
		ptr_feature_vector.push_back (&lfeatures_class::feature_11);
		ptr_feature_vector.push_back (&lfeatures_class::feature_12);
		ptr_feature_vector.push_back (&lfeatures_class::feature_13);
		ptr_feature_vector.push_back (&lfeatures_class::feature_14);
		ptr_feature_vector.push_back (&lfeatures_class::feature_15);
		ptr_feature_vector.push_back (&lfeatures_class::feature_16);
		ptr_feature_vector_cmp.push_back (&lfeatures_class::feature_01_comparative);

		feature_size = FEATURE_SET_0;
    }
    else if (feature_set == 1 )
    {
		ptr_feature_vector.push_back (&lfeatures_class::feature_01);
		ptr_feature_vector.push_back (&lfeatures_class::feature_02);
		ptr_feature_vector.push_back (&lfeatures_class::feature_03);
		ptr_feature_vector.push_back (&lfeatures_class::feature_04);
		ptr_feature_vector.push_back (&lfeatures_class::feature_05);
		ptr_feature_vector.push_back (&lfeatures_class::feature_06);
		ptr_feature_vector.push_back (&lfeatures_class::feature_07);
		ptr_feature_vector.push_back (&lfeatures_class::feature_08);
		ptr_feature_vector.push_back (&lfeatures_class::feature_09);
		ptr_feature_vector.push_back (&lfeatures_class::feature_10);
		ptr_feature_vector.push_back (&lfeatures_class::feature_11);
		ptr_feature_vector.push_back (&lfeatures_class::feature_12);
		ptr_feature_vector.push_back (&lfeatures_class::feature_13);
		ptr_feature_vector.push_back (&lfeatures_class::feature_16);

		feature_size = FEATURE_SET_1;
	}
}

////-------------------------------------------------------------------------------------------------------------------

void lfeatures_class::compute_descriptor(vector<Point3D_container> &all_laserfeat_cluster, vector< vector<Real> > &descriptor)
{
    //	descriptor = vector< vector <Real> > (all_laserfeat_cluster.size(), vector<Real>(feature_size) );
    descriptor = vector< vector<Real> > (all_laserfeat_cluster.size(), vector<Real>(0));

    if (feature_set == 0)
    {
        for (uint c = 0 ; c < all_laserfeat_cluster.size(); c++)
		{
            //			uint fidx = 0;
			//~ do standard feats
            for (uint f = 0; f < ptr_feature_vector.size(); f++)
			{
				Real value;
				char ret = CALL_MEMBER_FN(*this, ptr_feature_vector[f]) (&all_laserfeat_cluster[c], &value);

                // error
                if (!ret)
				{
                    printf("lfeatures.cpp: Error in feature computation SMP.  Check data validity [f: %d c: %d]\n",f,c);
					exit(1);
				}

				descriptor[c].push_back(value);
				//				descriptor[c][fidx] = value;
				//				fidx++;
			}

			//~  do rel feats
            for (uint f = 0; f < ptr_feature_vector_cmp.size(); f++)
			{
				Real value;
				char ret = CALL_MEMBER_FN_CMP(*this, ptr_feature_vector_cmp[f]) (all_laserfeat_cluster, c, &value);

                // error
                if (ret == 0)
				{
                    printf("lfeatures.cpp: Error in feature computation CMP. Check data validity [f: %d c: %d]\n", f, c);
					exit(1);
				}

				descriptor[c].push_back(value);
				//descriptor[c][fidx] = value;
				//fidx++;
			}

		}

    }
    else if (feature_set == 1)
    {
        for (uint c = 0 ; c < all_laserfeat_cluster.size(); c++)
		{
			//~ do standard feats
            for (uint f = 0; f < FEATURE_BASIS; f++)
			{
				Real value;
				char ret = CALL_MEMBER_FN(*this, ptr_feature_vector[f]) (&all_laserfeat_cluster[c], &value);

                // error
                if (ret == 0)
				{
                    printf("lfeatures.cpp: Error in feature computation SMP.  Check data validity [f: %d c: %d]\n", f, c);
					exit(1);
				}

				descriptor[c].push_back(value);
			}

			/*
			    case 63
            featuresVector = [featuresVector([1:3,6:15],:); ...
            featuresVector([1:3,6:15],:)./repmat(featuresVector(16,:),[13 1]); ... % dist normalized
            featuresVector([1:3,6:15],:).*repmat(featuresVector(16,:),[13 1]);
            featuresVector([2:3,6:15],:)./repmat(featuresVector(1,:),[12 1]); ... % point normalized
            featuresVector([2:3,6:15],:).*repmat(featuresVector(1,:),[12 1])];
			 */

			// EXTRA FEATURES
            float distance = 1.0;
			feature_16(&all_laserfeat_cluster[c], &distance);

			for (uint i = 0; i < FEATURE_BASIS; i ++)
            {
				descriptor[c].push_back(descriptor[c][i] / distance);
            }

            for (uint i = 0; i < FEATURE_BASIS; i ++)
            {
                descriptor[c].push_back(descriptor[c][i] * distance);
            }

            for (uint i = 1; i < FEATURE_BASIS; i ++)
            {
                descriptor[c].push_back(descriptor[c][i] / descriptor[c][0]);
            }

            for (uint i = 1; i < FEATURE_BASIS; i ++)
            {
                descriptor[c].push_back(descriptor[c][i] * descriptor[c][0]);
            }
		}

    }
    else
    {
		printf("[LFEATURES] Wrong feature_set value");
		exit(-1);
	}

	//~ all segments
}



////-------------------------------------------------------------------------------------------------------------------

/// feature 01: Num of points
int lfeatures_class::feature_01(Point3D_container *laserfeat_cluster, Real *out)
{		
    *out = laserfeat_cluster->pts.size();
    return 1;
}


////-------------------------------------------------------------------------------------------------------------------
/// feature 02: std from centroid (in 2D x and y)
int lfeatures_class::feature_02(Point3D_container *laserfeat_cluster, Real *out)
{
	Point3D_str pts_cog;	
	char ret = 1;

	// cog	
    laserfeat_cluster->compute_cog(&pts_cog);

	// compute vectorial std from 
	double s_sum = 0;

    for (uint i = 0; i < laserfeat_cluster->pts.size(); i++ )
	{
        double val = distance_L2_XY(&pts_cog, &laserfeat_cluster -> pts[i]);
        s_sum += val * val;
	}

    // error
    if (s_sum == 0)
    {
		ret = 0;
    }

    *out  = sqrt(s_sum / (double)laserfeat_cluster -> pts.size());

    return ret;
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 03: Mean average deviation from median (in 2D xy)
int lfeatures_class::feature_03(Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;
	double s_sum = 0;
	Point3D_str pts_median;

    vector<double> pts_coordx,pts_coordy;

	// take x
    laserfeat_cluster->get_coords(pts_coordx, GEOMETRY_COORD_X);
	// sort data
    gsl_sort(&pts_coordx[0], 1, pts_coordx.size());
	// median
	pts_median.x = gsl_stats_median_from_sorted_data (&pts_coordx[0], 1, pts_coordx.size());
	// take y
    laserfeat_cluster->get_coords(pts_coordy, GEOMETRY_COORD_Y);
	// sort data
    gsl_sort(&pts_coordy[0], 1, pts_coordy.size());
	// median
    pts_median.y = gsl_stats_median_from_sorted_data(&pts_coordy[0], 1, pts_coordy.size());
	// project on xy plane
	pts_median.z = 0;

    for (uint i = 0; i < laserfeat_cluster->pts.size(); i++)
    {
        double val = distance_L2_XY (&pts_median, &laserfeat_cluster -> pts[i]);
		s_sum += val * val;	
	}

    // error
    if (s_sum == 0)
    {
		ret = 0;
    }

	//printf("feature 03: median x %f y %f , sum %f \n",pts_median.x, pts_median.y, s_sum);

    *out = sqrt(s_sum / (double)laserfeat_cluster-> pts.size());

    return ret;
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 04: width
int lfeatures_class::feature_04(Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;

	//~ endpoints
    int lastidx = laserfeat_cluster->pts.size() - 1;

	//~ 2 pts are enough
    if (laserfeat_cluster->pts.size() >= 2)
    {
        *out = distance_L2_XY(&laserfeat_cluster->pts[0] , &laserfeat_cluster->pts[lastidx]);
    }
    // error
	else
    {
		ret = 0;
    }

    return ret;
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 05: circularity (in 2D plane xy)
int lfeatures_class::feature_05(Point3D_container *laserfeat_cluster, Real *out)
{	 
	char ret = 1;

	Point3D_str circle_param;
    double s_sum = 0;

	//~ 3 pts to fit a circle
    if (laserfeat_cluster-> pts.size() >= 3)
	{
		// get cirlce parameters xc yc rc (rc is stored in Z var)
		get_circle_param( laserfeat_cluster, &circle_param);

		double xc = circle_param.x;
		double yc = circle_param.y;
		double rc = circle_param.z;

		//~ avoid nans and aligned points
        if (isnan(rc) || rc >= L_HUGENUM)
		{
			s_sum = L_HUGENUM;
		}
		else
		{
			// residual sum
            for (uint i = 0; i < laserfeat_cluster->pts.size(); i++)
			{
				double dx = xc - laserfeat_cluster->pts[i].x;
				double dy = yc - laserfeat_cluster->pts[i].y;
				double residual = rc - sqrt(dx * dx + dy * dy);
				s_sum += residual*residual;
			}	
		}

		ret = 1;
	}
    // error
	else
    {
		ret = 0;
    }

	*out = s_sum;

    return ret;
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 06: radius fit circle  (in 2D plane xy)
int lfeatures_class::feature_06(Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;		
	Point3D_str circle_param;

	//~ 3 pts to fit a circle
    if (laserfeat_cluster-> pts.size() >= 3)
	{
		// get cirlce parameters xc yc rc (rc is stored in Z var)
		get_circle_param( laserfeat_cluster, &circle_param);

        if (isnan(circle_param.z)  || circle_param.z >= L_HUGENUM)
        {
			circle_param.z = L_HUGENUM;
        }

		*out = circle_param.z;		
	}
    // error
	else
    {
		ret = 0;
    }

    return ret;
}


////-------------------------------------------------------------------------------------------------------------------


/// feature 07: boundary length (in 2D plane xy)
int lfeatures_class::feature_07(Point3D_container *laserfeat_cluster, Real *out)
{
	double s_sum = 0;
	char ret = 1;	

    if (laserfeat_cluster->pts.size() >= 2)
	{
        for (uint i = 0; i < laserfeat_cluster->pts.size() - 1; i++)
		{
			//~ disregard z
			laserfeat_cluster->pts[i].z = 0;
			laserfeat_cluster->pts[i+1].z = 0;
			s_sum += distance_L2_XY (&laserfeat_cluster->pts[i] , &laserfeat_cluster->pts[i+1]);
		}
    }
    // error
    else
    {
        ret = 0;
    }

    *out = s_sum;

    return ret;
}


////-------------------------------------------------------------------------------------------------------------------


/// feature 08:   boundary regularity (in 2D plane xy)
int lfeatures_class::feature_08(Point3D_container *laserfeat_cluster, Real *out)
{
    char ret = 1;

    if (laserfeat_cluster->pts.size() >= 3)
    {
        vector<double> dist ( laserfeat_cluster->pts.size() - 1);

        for (uint i = 0; i <laserfeat_cluster->pts.size() - 1; i++)
        {
            dist[i] = distance_L2_XY (&laserfeat_cluster->pts[i] , &laserfeat_cluster->pts[i + 1]);
        }

		*out = gsl_stats_sd (&dist[0], 1, dist.size());
    }
    // error
    else
    {
        ret = 0;
    }

    return ret;
}


////-------------------------------------------------------------------------------------------------------------------

/// feature 09: mean curvature  (in 2D plane xy)
int lfeatures_class::feature_09(Point3D_container *laserfeat_cluster, Real *out)
{	 
	//~ the result is the inverse of the fitted circle radius
	char ret = 1;
	double k_sum = 0;
	double area;

    if (laserfeat_cluster->pts.size() > 2)
    {
        vector<double> d(3);

        for (uint i = 1; i <laserfeat_cluster->pts.size() - 1; i++)
		{
			d[0] =  distance_L2_XY (&laserfeat_cluster->pts[i - 1], &laserfeat_cluster->pts[i]);
			d[1] =  distance_L2_XY (&laserfeat_cluster->pts[i], &laserfeat_cluster->pts[i+1]);
			d[2] =  distance_L2_XY (&laserfeat_cluster->pts[i - 1], &laserfeat_cluster->pts[i + 1]);

            sort(d.begin(), d.end());

			//~ numerically stable Hero's formula
            area = (d[0] + d[1] + d[2])  * (d[2] - (d[0] - d[1]))
                   * (d[2] + (d[0] - d[1]))  * (d[0] + (d[1] - d[2]))
                   + L_EPSNUM;

			//~ no curvature
            if (area < 0)
            {
				area = 0;
            }

			area = 0.25 * sqrt(area);
			double denom = d[0]*d[1]*d[2] + L_EPSNUM;
			k_sum += (4.0 * area) / denom;

		}

		*out =  k_sum / ((double)laserfeat_cluster->pts.size());
    }
    // error
    else
    {
        ret = 0;
    }

    return ret;
}


////-------------------------------------------------------------------------------------------------------------------

/// feature 10: mean angular difference on convex hull border (in 2D plane xy)
int lfeatures_class::feature_10(Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;
	double res;
	double sc_prd, n0, n1, div;
	double ang_sum = 0;
	Point3D_str pt0, pt1;
	Point3D_str origin;	

	origin.x = 0;
	origin.y = 0;
	origin.z = 0;

    if (laserfeat_cluster->pts.size() > 2)
	{
        for (uint i = 1; i <laserfeat_cluster->pts.size() - 1; i++)
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
            if (div > 0)
			{
				// scalar product
				sc_prd = pt0.x * pt1.x + pt0.y * pt1.y;
				res = sc_prd / div;

				//~ numerical instabilities
                if (res > 0.0)
                {
                    res -= L_EPSNUM;
                }
                else if (res < 0.0)
                {
                    res += L_EPSNUM;
                }

                if (fabs(res) > 1.0)
				{
					printf("ERROR: memory corruption in feature10\n") ;
					exit(1);
				}

                ang_sum += acos(res);
			}
		}

		*out = ang_sum / (double)(laserfeat_cluster->pts.size() - 2);
    }
    // error
    else
    {
        ret = 0;
    }

    return ret;
}

////-------------------------------------------------------------------------------------------------------------------

/** feature 11:  aspect ratio1: GEOMETRY_MAX(std_x,std_y) / GEOMETRY_MIN(std_x,std_y)
    This initially had a problem due to the call to get_coords(pts, coord) . Fixed.
    @see LSL_Point3D_container::get_coords (vector< double > &pts_coord, char coord_sel)
 */
int lfeatures_class::feature_11(Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;

    double GEOMETRY_MIN_v, GEOMETRY_MAX_v, std1, std2;
    vector<double> pts_coordx,pts_coordy;

    laserfeat_cluster->get_coords(pts_coordx, GEOMETRY_COORD_X);
	//printf("feature 11 size: cluster %d pts_coord %d\n",  laserfeat_cluster->pts.size() , pts_coord.size());

    std1 = gsl_stats_sd(&pts_coordx[0], 1, laserfeat_cluster->pts.size() -1) ;
	//printf("std1 %f\n", std1);

    laserfeat_cluster->get_coords(pts_coordy, GEOMETRY_COORD_Y);
    std2 = gsl_stats_sd(&pts_coordy[0], 1, laserfeat_cluster->pts.size() -1) ;
	//printf("std2 %f\n", std2);


	GEOMETRY_MAX_v = GEOMETRY_MAX(std1, std2);	
	GEOMETRY_MIN_v = GEOMETRY_MIN(std1, std2);

	*out = (1.0 + GEOMETRY_MIN_v) / ( 1.0 + GEOMETRY_MAX_v) ;
	//printf("feature 11: %f\n",*out);

    return ret;
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 12:  kurtosis (2D: plane xy)
int lfeatures_class::feature_12(Point3D_container *laserfeat_cluster, Real *out)
{
    char ret = 1;
	Point3D_str pts_cog;
	double s_sum = 0;

	//~ at least 2pt
    if (laserfeat_cluster->pts.size() >= 2)
    {
        // get COG
        laserfeat_cluster->compute_cog(&pts_cog);

        // compute kurtosis
        for (uint i = 0 ; i < laserfeat_cluster -> pts.size(); i++)
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
    }
    // error
    else
    {
        ret = 0;
    }

    return ret;
}

////-------------------------------------------------------------------------------------------------------------------

/// feature 13: linearity
int lfeatures_class::feature_13(Point3D_container *laserfeat_cluster, Real *out)
{
	char ret = 1;	
	double s_sum = 0;

    //~ at least 2pt
    if (laserfeat_cluster-> pts.size() >= 2)
	{
		Point3D_str line_param, pts_cog;

		// get line parameters 
		get_line_param( laserfeat_cluster, &line_param);

		double m = line_param.x;
		double q = line_param.y;

		//~ vertical line
        if (fabs(m) > L_HUGENUM)
		{
			laserfeat_cluster -> compute_cog(&pts_cog);

			// residual sum
            for (uint i = 0; i < laserfeat_cluster->pts.size(); i++)
			{
				double dx = laserfeat_cluster->pts[i].x - pts_cog.x;
				s_sum += dx * dx;
			}	
		}
		else
		{
			// residual sum
            for (uint i = 0; i < laserfeat_cluster->pts.size(); i++)
			{
				double rr = (m * laserfeat_cluster->pts[i].x + q  - laserfeat_cluster->pts[i].y);
				s_sum += rr*rr;
			}
        }

        //~ printf("%f\n",s_sum);
        *out =  s_sum;
    }
    // error
    else
    {
        ret = 0;
    }

    return ret;
}


////-------------------------------------------------------------------------------------------------------------------


/// feature 14: circularity normed (in 2D plane xy)
int lfeatures_class::feature_14(Point3D_container *laserfeat_cluster, Real *out)
{
	Real out1;

	char ret = feature_05(laserfeat_cluster, &out1);
    *out = out1 / (double)laserfeat_cluster->pts.size();

    return ret;
}




////-------------------------------------------------------------------------------------------------------------------


/// feature 15: linearity normed (in 2D plane xy)
int lfeatures_class::feature_15(Point3D_container *laserfeat_cluster, Real *out)
{
	Real out1;

	char ret = feature_13(laserfeat_cluster, &out1);
	*out = out1 / (double)laserfeat_cluster-> pts.size();

    return ret;
}



////-------------------------------------------------------------------------------------------------------------------

/// feature 16: distance
int lfeatures_class::feature_16(Point3D_container *laserfeat_cluster, Real *out)
{
    char ret = 1;

	// origin
	Point3D_str origin, pts_cog;	

	origin.x = 0;
	origin.y = 0;
	origin.z = 0;

	// get COG
	laserfeat_cluster->compute_cog(&pts_cog);

	// compute distance
	double dist = distance_L2_XY(&pts_cog, &origin);

    *out = dist;

    return ret;
}


////-------------------------------------------------------------------------------------------------------------------


/// distance nearest cluster (via cogs)
int lfeatures_class::feature_01_comparative(vector<Point3D_container> &all_laserfeat_cluster, uint curidx, Real *out)
{		
    char ret = 1;
	float bestdist = FLT_MAX;
	Point3D_str pts_cog;	

	all_laserfeat_cluster[curidx].compute_cog(&pts_cog);

    for (uint i = 0 ; i < all_laserfeat_cluster.size(); i++)
	{
        if (i != curidx)
		{
			Point3D_str pts_curcog;	
			all_laserfeat_cluster[i].compute_cog(&pts_curcog);
			double curdist = distance_L2_XY(&pts_cog, &pts_curcog);

            if (curdist < bestdist)
            {
				bestdist = curdist;
            }
		}
	}

    *out = bestdist;

    return ret;
}

