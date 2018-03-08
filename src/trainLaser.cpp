/**
 * Trains a boosted classifier and saves the model to data/trained_boost.xml
 * TODO: input from command line the file and the number of weak classifiers to be trained
 */

#include <cstdlib>
#include <opencv2/ml.hpp>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <vector>


//using namespace cv;
//using cv::TrainData;
//using cv::Boost;
//using cv::BoostParams;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TrainLaser");


    ros::NodeHandle nh("~");

    if (nh.hasParam("read_path") && nh.hasParam("write_path"))
    {
        std::string read_path;
        std::string write_path;
        nh.getParam("read_path", read_path);
        nh.getParam("write_path", write_path);

        ROS_INFO("[TrainLaser] - Reading from %s", read_path.c_str());

        // STEP 2. Opening the file

        //1. Declare a structure to keep the data

        cv::Ptr<cv::ml::TrainData> cvml;
//        Parameters
//            filename	The input file name
//            headerLineCount	The number of lines in the beginning to skip; besides the header, the function also skips empty lines and lines staring with #
//            responseStartIdx	Index of the first output variable. If -1, the function considers the last variable as the response
//            responseEndIdx	Index of the last output variable + 1. If -1, then there is single response variable at responseStartIdx.
//            varTypeSpec	The optional text string that specifies the variables' types. It has the format ord[n1-n2,n3,n4-n5,...]cat[n6,n7-n8,...]. That is, variables from n1 to n2 (inclusive range), n3, n4 to n5 ... are considered ordered and n6, n7 to n8 ... are considered as categorical. The range [n1..n2] + [n3] + [n4..n5] + ... + [n6] + [n7..n8] should cover all the variables. If varTypeSpec is not specified, then algorithm uses the following rules:

//                all input variables are considered ordered by default. If some column contains has non- numerical values, e.g. 'apple', 'pear', 'apple', 'apple', 'mango', the corresponding variable is considered categorical.
//                if there are several output variables, they are all considered as ordered. Error is reported when non-numerical values are used.
//                if there is a single output variable, then if its values are non-numerical or are all integers, then it's considered categorical. Otherwise, it's considered ordered.

//            delimiter	The character used to separate values in each line.
//            missch	The character used to specify missing measurements. It should not be a digit. Although it's a non-numerical value, it surely does not affect the decision of whether the variable ordered or categorical.

//        Note
//            If the dataset only contains input variables and no responses, use responseStartIdx = -2 and responseEndIdx = 0. The output variables vector will just contain zeros.

        cvml  = cv::ml::TrainData::loadFromCSV(read_path.c_str(), 0, 0, -1);

        ROS_INFO("[TrainLaser] - Training ...");

        // STEP 4. The training

        //1. Declare the classifier

        cv::Ptr<cv::ml::Boost> boost;

        ROS_INFO("[TrainLaser] - Create boost");

        boost = cv::ml::Boost::create();

        // Set adabooost parameters
        //   int boost_type, int weak_count, double weight_trim_rate, int max_depth, bool use_surrogates, const float* priors
        //  cv::ml::Boost::Params(cv::CvBoost::REAL, 100, 0, 1, false, 0)

        ROS_INFO("[TrainLaser] - Set parameters");
        boost->setBoostType(cv::ml::Boost::REAL);
        boost->setWeakCount(100);
        boost->setWeightTrimRate(0);
        boost->setMaxDepth(1);
        boost->setUseSurrogates(false);
        //boost->setPriors(cv::Mat0);

        //2. Train it with max tree depth 100


//        int  	boost_type,
//                int  	weak_count,
//                double  	weight_trim_rate,
//                int  	max_depth,
//                bool  	use_surrogates,
//                const float *  	priors
        //bool success = boost->train(&cvml, cv::ml::Boost::Params(cv::CvBoost::REAL, 100, 0, 1, false, 0), false);
        ROS_INFO("[TrainLaser] - Train");
        bool success = boost->train(cvml, 0);
        if (success)
        {

             ROS_INFO("[TrainLaser] - Training Success!!");
        }
        else
        {

            ROS_INFO("[TrainLaser] - Training Failed :(");
            return EXIT_FAILURE;
        }


        // STEP 5. Calculating the testing and training error
        // 1. Declare a couple of vectors to save the predictions of each sample

        // std::vector<float> train_responses, test_responses;
        // 2. Calculate the rate of the training error in %
        //    float fl1 = boost->calcError(cvml, true, &train_responses);

        //    CvTrainTestSplit cvtts2(0.5f, true);
        //    cvml.set_train_test_split(&cvtts2);

        //    // 3. Calculate the rate of the testing error in %
        //    float fl2 = boost.calc_error(&cvml, CV_TEST_ERROR, &test_responses);

        //    printf("Error train %f %% \n", fl1);
        //    printf("Error test %f %% \n", fl2);

        // STEP 6. Save your classifier
        // Save the trained classifier
        boost->save(write_path.c_str());
    }
    else
    {
        ROS_ERROR("[TrainLaser] Need to set the parameters <read_path> and <write_path> in order to load the anotation and to save trained files.");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
