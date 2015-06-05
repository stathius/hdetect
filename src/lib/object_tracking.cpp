#include <hdetect/lib/object_tracking.hpp>
                  
//using namespace std;
using std::deque;
using std::map;
using NEWMAT::Matrix;
using NEWMAT::IdentityMatrix;

using namespace Header;

namespace ObjectTracking
{
  float max_mah_dist;
  float max_euc_dist;

  int init_id;

  float new_object_score;
  float predict_score;
  float update_score;
}

void ObjectTracking::loadCfg(string cfg)
{

  max_mah_dist = 2.3;
  max_euc_dist = 3.3;

  init_id = 1;

  new_object_score = 4.0;
  predict_score = -1.0;
  update_score = 1.0;
}
void ObjectTracking::loadCfg(detectorParameters params)
{
  max_mah_dist = params.max_mah_dist;
  max_euc_dist = params.max_euc_dist;

  init_id =  params.init_id;

  new_object_score = params.new_object_score;
  predict_score = params.predict_score;
  update_score = params.update_score;
}

void ObjectTracking::eliminate(deque<Human> &humans)
{
  for (int i = 0; i < (int)humans.size(); i++)
  {
    //if(i==1)
    //fprintf(stderr, "delete human %d - Score %f\n",i+1,humans[i].score);
    if (humans[i].score < 0.0 || curTimestamp - humans[i].preTimestamp > 5)
    {
      humans.erase(humans.begin() + i);
      i--;
    }
  }
}

void ObjectTracking::predict(deque<Human> &humans)
{
  float deltaT = curTimestamp - preTimestamp;

  Matrix A = Matrix(4, 4);
  A << 1 << 0 << deltaT << 0 <<
       0 << 1 << 0 << deltaT <<
       0 << 0 << 1 << 0 <<
       0 << 0 << 0 << 1;

  Matrix Q = Matrix(4, 4);
  Q << 0.16 * deltaT << 0 << 0 << 0 <<
       0 << 0.16 * deltaT << 0 << 0 <<
       0 << 0 << 0.2025 * deltaT << 0 <<
       0 << 0 << 0 << 0.2025 * deltaT;

  for (uint i = 0; i < humans.size(); i++)
  {
    humans[i].score += predict_score * deltaT;
    humans[i].state = A * humans[i].state;
    humans[i].cov = A * humans[i].cov * A.t() + Q;
    //fprintf(stderr, "predict (ID, score, deltaT) = (%d, %.2f, %.2f) \n", humans[i].id, humans[i].score, deltaT);
  }
}

void ObjectTracking::pair(deque<Human> &humans, deque<Observation> &observations, map<int, int> &pairs)
{
  if (humans.size() == 0 || observations.size() == 0)
  {
    //fprintf(stderr, "No observations");
    return;
  }

  // Matching Matrix
  Matrix MahDis = Matrix(observations.size(), humans.size());
  Matrix EucDis = Matrix(observations.size(), humans.size());

  for (uint i = 0; i < observations.size(); i++)
  {
    for (uint j = 0; j < humans.size(); j++)
    {
      // Mahalanobis Distance and Euclidean Distance
      MahDis(i + 1, j + 1) = calculateMahDis(observations[i], humans[j]);
      EucDis(i + 1, j + 1) = calculateEucDis(observations[i], humans[j]);
      //fprintf(stderr, "observation %d, humans %d, minEuc = %.2f, minMah = %.2f \n",i, j, EucDis(i + 1, j + 1), MahDis(i + 1, j + 1));
    }
  }

  int pairNum = 0;

  //fprintf(stderr, "---------- Pair Start ----------\n");
  while (true)
  {
    // Get the row and the col of the minimum value in the matrix
    int row = 0;
    int col = 0;

    // int rowE = 0;
    // int colE = 0;
    //float minEuc = 10;

    float minMah = MahDis.Minimum2(row, col);
    float minEuc = EucDis(row, col);

    // float minEuc_2 = EucDis.Minimum2(rowE, colE);
    // float minMah_2 = MahDis(rowE, colE);
    // float minMah = MahDis(row, col);

    // float minMah = MahDis(row, col);
    // If the euclidean distance is different from the mahalanobis one
    // discard the pairing
    //        if (row != rowE || col != colE)
    //        {
    //          if(minEuc > 2.0)
    //            minEuc = 10;
    //        }

    //fprintf(stderr, "row %d, col %d, minEuc = %.2f, minMah = %.2f \n",row, col, minEuc, minMah);

    // Observation index
    int i = row - 1;

    // Human index
    int j = col - 1;

    // Pair observation and human only if distance is smaller than MAX_MAH_DIS and MAX_EUC_DIS
    if (minMah < max_mah_dist)
    {
      if (minEuc < max_euc_dist)
      {
        pairs[i] = j;
        pairNum += 1;

        // Puts the row and column selected to infinity to avoid double pairing
        EucDis.Row(row) = std::numeric_limits<float>::infinity();
        EucDis.Column(col) = std::numeric_limits<float>::infinity();
        MahDis.Row(row) = std::numeric_limits<float>::infinity();
        MahDis.Column(col) = std::numeric_limits<float>::infinity();

        //fprintf(stderr, "Success = %d %d, minEuc = %.2f, minMah = %.2f, cov = %.2f %.2f \n",
        //        i, humans[j].id, minEuc, minMah, humans[j].cov(1, 1), humans[j].cov(2, 2));
      }
      else
      {
        //     fprintf(stderr, "Euc Fail -- Hum id: %d Obs: %d, minMah = %.2f - minEuc = %.2f \n", humans[j].id, i, minMah, minEuc);
        EucDis(row, col) = std::numeric_limits<float>::infinity();
        MahDis(row, col) = std::numeric_limits<float>::infinity();
      }
    }
    else
    {
      //fprintf(stderr, "Euc Fail = Obs: %d Hum id: %d, minEuc = %.2f \n", i, humans[j].id, minEuc);
      break;
    }

    if (pairNum == fmin(observations.size(), humans.size()))
    {
      break;
    }
  }
  //fprintf(stderr, "----------  Pair End  ----------\n");
}

void ObjectTracking::update(deque<Human> &humans, deque<Observation> &observations, map<int, int> &pairs)
{
  if (observations.size() == 0)
  {
    return;
  }

  float deltaT = curTimestamp - preTimestamp;

  Matrix R = Matrix(4, 4);

  R << 0.09 << 0 << 0 << 0 <<
       0 << 0.09 << 0 << 0 <<
       0 << 0 << 0.36 << 0 <<
       0 << 0 << 0 << 0.36;

  deque<bool> unpairs;
  unpairs.resize(observations.size(), true);

  // For Pairing Result
  for (map<int, int>::iterator it = pairs.begin(); it != pairs.end(); it++)
  {
    // Observation index
    int i = it->first;

    // Human index
    int j = it->second;

    observations[i].state(3) = (observations[i].state(1) - humans[j].preState(1)) / (curTimestamp - humans[j].preTimestamp);
    observations[i].state(4) = (observations[i].state(2) - humans[j].preState(2)) / (curTimestamp - humans[j].preTimestamp);

    Matrix H = IdentityMatrix(4);
    Matrix Y = observations[i].state - H * humans[j].state;
    Matrix S = H * humans[j].cov * H.t() + R;
    Matrix K = humans[j].cov * H.t() * S.i();
    Matrix I = IdentityMatrix(K.Ncols());

    // Maintain score if only detected by laser
    humans[j].score -=  predict_score * deltaT;

    // Increase score if detected by camera
    if (observations[i].camera_detected == true)
    {
      // humans[j].score -= 0.01 * PREDICT_OBJECT_SCORE * deltaT;
      humans[j].score += ((update_score - (calculateEucDis(observations[i], humans[j]) / max_euc_dist)) +
                          (update_score - (calculateMahDis(observations[i], humans[j]) / max_mah_dist)) * 2) * deltaT;
    }

    humans[j].state = humans[j].state + K * Y;
    humans[j].cov = (I - K * H) * humans[j].cov;

    humans[j].preState = humans[j].state;
    humans[j].preTimestamp = curTimestamp;

    unpairs[i] = false;
    //fprintf(stderr, "paired (observation, ID) = (%d, %d) \n", i, humans[j].id);
  }

  // For New Observations Result
  for (uint i = 0; i < observations.size(); i++)
  {
    // Only the observation detected by image can be put into the human list
    if (unpairs[i] == true && observations[i].camera_detected == true)
    {
      humans.push_back(Human(init_id, new_object_score, observations[i].state, R, curTimestamp));
      init_id += 1;
      pairs[i] = humans.size() - 1;
      // fprintf(stderr, "new (observation, ID) = (%d, %d) \n", i, humans[pairs[i]].id);
    }
  }
}

float ObjectTracking::calculateMahDis(Observation &observation, Human &human)
{
  float sum1 = (human.state(1) - observation.state(1)) * (human.state(1) - observation.state(1)) / human.cov(1, 1);
  float sum2 = (human.state(2) - observation.state(2)) * (human.state(2) - observation.state(2)) / human.cov(2, 2);
  return sqrt(sum1 + sum2);
}

float ObjectTracking::calculateEucDis(Observation &observation, Human &human)
{
  float sum1 = (human.state(1) - observation.state(1)) * (human.state(1) - observation.state(1));
  float sum2 = (human.state(2) - observation.state(2)) * (human.state(2) - observation.state(2));

  return sqrt(sum1 + sum2);
}
