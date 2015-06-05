/**
 * Trains a boosted classifier and saves the model to data/trained_boost.xml
 * TODO: input from command line the file and the number of weak classifiers to be trained
 */

#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
#include <ros/ros.h>
#include <vector>


//using namespace cv;
using cv::TrainData;
using cv::Boost;
using cv::BoostParams;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TrainLaser");

    std::string abs_path;
    ros::NodeHandle nh;

    if (nh.hasParam("pkg_path"))
    {
        ros::param::get("/pkg_path",abs_path);
    }
    else
    {
        ROS_ERROR("[TRAIN LASER] Parameter pkg_path (absolute package path) not found.");
    }


    std::string read_path = abs_path + argv[1];
    std::string write_path = abs_path + "/data/trained_boost.xml";

    printf("Reading from %s\n", read_path.c_str());

    /// STEP 2. Opening the file
    //1. Declare a structure to keep the data
    TrainData cvml;

    //2. Read the file
    cvml.read_csv(read_path.c_str());

    //3. Indicate which column is the response

    // Change the type of the first column to categorical
    cvml.change_var_type(0, CV_VAR_CATEGORICAL);

    // Set the the first column as the response
    cvml.set_response_idx(0);

//    // From col 0 to col N-1, 0 means NUMERICAL and 1 means CATEGORICAL
//    for (int j = 0; j < cvml.get_var_types()->cols; j++)
//    {
//        printf("%-4d", cvml.get_var_type(j));
//    }
//    printf("\n");

//    // From col 0 to col N-1, 0 means NUMERICAL and 1 means CATEGORICAL
//    // Col N-1 is the response column
//    Mat tp = cvml.get_var_types();
//    for (int j = 0; j < tp.cols; j++)
//    {
//        printf("%-4d", tp.at<uchar>(0,j));
//    }
//    printf("\n");


//    Mat val = cvml.get_values();
//    for (int i = 0; i < val.rows; i++)
//    {
//        printf("%-4d", i);

//        for (int j = 0; j < val.cols; j++)
//        {
//            printf("%-7.2f", val.at<float>(i, j));
//        }

//        printf("\n");
//    }

//    Mat resp = cvml.get_responses();
//    for (int i = 0; i < resp.rows; i++)
//    {
//        printf("%-4d ",i);

//        for (int j = 0; j < resp.cols; j++)
//        {
//            printf("%2.2f", resp.at<float>(i,j));
//        }

//        printf("\n");
//    }


    /// STEP 3. Splitting the samples
    //1. Select 80% for the training
    float portion = 1.0;
    CvTrainTestSplit cvtts(portion, true);

    //2. Assign the division to the data
    cvml.set_train_test_split(&cvtts);


    printf("\nTraining ...\n");
    // STEP 4. The training
    //1. Declare the classifier
    Boost boost;

    //2. Train it with max tree depth 100
    bool success = boost.train(&cvml, BoostParams(CvBoost::REAL, 100, 0, 1, false, 0), false);
    if (success)
    {
        printf("Training Success ...\n");
    }
    else
    {
        printf("Training Failed ...\n");
        return EXIT_FAILURE;

    }

    /// STEP 5. Calculating the testing and training error
    // 1. Declare a couple of vectors to save the predictions of each sample
    std::vector<float> train_responses, test_responses;

    // 2. Calculate the rate of the training error in %
    float fl1 = boost.calc_error(&cvml, CV_TRAIN_ERROR, &train_responses);

    CvTrainTestSplit cvtts2(0.5f, true);
    cvml.set_train_test_split(&cvtts2);

    // 3. Calculate the rate of the testing error in %
    float fl2 = boost.calc_error(&cvml, CV_TEST_ERROR, &test_responses);

    printf("Error train %f %% \n", fl1);
    printf("Error test %f %% \n", fl2);

    // STEP 6. Save your classifier
    // Save the trained classifier
    boost.save(write_path.c_str(), "boost");

    return EXIT_SUCCESS;
}
