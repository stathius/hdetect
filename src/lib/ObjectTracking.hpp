#ifndef OBJECTDETECTION_HPP
#define OBJECTDETECTION_HPP

#include <newmat/newmat.h>
#include <stdio.h>
#include <string>
#include <deque>
#include <map>

#include "Header.hpp"
#include "Human.hpp"
#include "Observation.hpp"

namespace ObjectTracking
{
    void loadCfg(string cfg);

    void eliminate(deque<Human> &humans);
    void predict(deque<Human> &humans);
    void pair(deque<Human> &humans, deque<Observation> &observations,map<int, int> &pairs);
    void update(deque<Human> &humans, deque<Observation> &observations, map<int, int> &pairs);

    float calculateMahDis(Observation &observation, Human &human);
    float calculateEucDis(Observation &observation, Human &human);
}

#endif // OBJECTDETECTION_HPP
