/**
 * Trains a boosted classifier and saves the model to data/trained_boost.xml
 * TODO: input from command line the file and the number of weak classifiers to be trained
 */
#include <cstdlib>
#include "opencv/cv.h"
#include "opencv/ml.h"
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

  /* STEP 2. Opening the file */
  //1. Declare a structure to keep the data
  CvMLData cvml;

  //2. Read the file
  cvml.read_csv("data/annotation_full.csv");

  //3. Indicate which column is the response

  // Change the type of the first column to categorical
  cvml.change_var_type(0, CV_VAR_CATEGORICAL);

  // Set the the first column as the response
  cvml.set_response_idx(0);

  /*
  for(int j=0; j < 18; j++)
      printf("%d\t", cvml.get_var_type(j));
  printf("\n");

   Mat tp = cvml.get_var_types();
   for(int j=0; j < tp.cols; j++)
       printf("%d\t", tp.at<uchar>(0,j));
   printf("\n");


  Mat val = cvml.get_values();
   for (int i=0; i < val.rows; i++ ) {
     printf("\n%d ",i);
     for(int j=0; j < val.cols; j++)
       printf("%f\t", val.at<float>(i,j));
   }

   Mat resp = cvml.get_responses();
    for (int i=0; i < resp.rows; i++ ) {
      printf("\n%d ",i);
      //for(int j=0; j < resp.cols; j++)
        printf("%f", resp.at<float>(i,0));
    }
    */

  // STEP 3. Splitting the samples
  //1. Select 1000 for the training
  CvTrainTestSplit cvtts(20, true);

  //2. Assign the division to the data
  cvml.set_train_test_split(&cvtts);



  printf("\nTraining ... \n");
  // STEP 4. The training
  //1. Declare the classifier
  CvBoost boost;

  //2. Train it with 100 features
  boost.train(&cvml, CvBoostParams(CvBoost::REAL, 50, 0, 1, false, 0), false);

  // STEP 5. Calculating the testing and training error
  // 1. Declare a couple of vectors to save the predictions of each sample
  std::vector<float> train_responses, test_responses;

  // 2. Calculate the training error
  float fl1 = boost.calc_error(&cvml,CV_TRAIN_ERROR,&train_responses);

  // 3. Calculate the test error
  float fl2 = boost.calc_error(&cvml,CV_TEST_ERROR,&test_responses);

  printf("Error train %f \n", fl1);
  printf("Error test %f \n", fl2);

  // STEP 6. Save your classifier
  // Save the trained classifier
  boost.save("data/trained_boost.xml", "boost");


  return EXIT_SUCCESS;

}
