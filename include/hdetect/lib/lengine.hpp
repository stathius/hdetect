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
#ifndef LENGINE_
#define LENGINE_H

#include <stdio.h>	
#include <vector>	
#include <string>	
#include <fstream>
#include <string.h>
#include <sys/time.h>

#include "lfeatures.hpp"

typedef struct
{
	double jumpdist, laser_range;
	int feature_set;
	char segonly, sanity, verbosity;
} lengine_params;


class laserscan_data
{
public:

	Point3D_container data;
	double timestamp;

	laserscan_data() { }
	laserscan_data(int num)
	{
		data = Point3D_container(num);
	}
};

class lengine
{
private:

	lengine_params params;
	std::vector<laserscan_data> laserscan;int get_breakpoint(std::vector<Point3D_str> &pts, int last_breaking_idx
	);int sanity_check(
			std::vector<std::vector<Real> > & descriptor);

	// made it public
	lfeatures_class *lfeatures;

	// UPM
	// instead of using the vector laserscan we will be using a
	// single laserscan_data object
	laserscan_data laserscan_single;

public:

	const static int feature_set_size[];

	int load_scandata(std::string fname);

	lengine(lengine_params param_in)
	{
		params = param_in;
	}

	void set_featureset();

	// UPM functions
	// Null constructor
	// lengine() { }

	void load_scandata(laserscan_data laserscan_input)
	{
		laserscan_single = laserscan_input;
	}

	int getScanSize()
	{
		return laserscan_single.data.pts.size();
	}

	int segmentscanJDC(std::vector<Point3D_container> &clusters);

	void computeFeatures(std::vector<Point3D_container> &clusters, std::vector<std::vector<float> > &descriptor);

};
#endif
