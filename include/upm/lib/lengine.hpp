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
  double dseg, sqjumpdist;
  int featuremix;
  char segonly, sanity, verbosity;
} sw_param_str;


class laserscan_data
{
public:

  LSL_Point3D_container data;
  double timestamp;

  laserscan_data() { }
  laserscan_data(int num)
  {
    data = LSL_Point3D_container(num);
  }
};

class lengine
{
private:
  sw_param_str params;
  std::vector<laserscan_data> laserscan;int get_breakpoint(std::vector<LSL_Point3D_str> &pts, int last_breaking_idx,
                                                           double sqjumpdist);int sanity_check(
  std::vector<std::vector<Real> > & descriptor);

  unsigned int fsz;

  // made it public
  LSL_lfeatures_class *lfeatures;

  // UPM
  // instead of using the vector laserscan we will be using a
  // signle laserscan_data object
  laserscan_data laserscan_single;

public:
  int load_scandata(std::string fname);

  lengine(sw_param_str param_in)
  {
    fsz = 0;
    params = param_in;
    params.sqjumpdist = params.dseg * params.dseg;
  }

  void set_featureset();

  // UPM functions

  lengine() { }

  void load_scandata(laserscan_data laserscan_input)
  {
    laserscan_single = laserscan_input;
  }

  int getScanSize()
  {
    return laserscan_single.data.pts.size();
  }

  int segmentscanJDC(std::vector<LSL_Point3D_container> &clusters);

  void computeFeatures(std::vector<LSL_Point3D_container> &clusters, std::vector<std::vector<float> > &descriptor);

};
#endif
