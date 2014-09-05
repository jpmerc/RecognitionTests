#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

#include <object_recognition.h>




// If it is the good name ('c' at the end of a name means it is a complete pointcloud)
bool compareName(const std::string& p_object, const std::string& p_result)
{
    bool validation = true;

    int size = p_object.size();
    if(p_result.size() < size)
    {
        size = p_result.size();
    }

    for(int i = 0; i < size; i++)
    {
        if(p_object[i] == '_' || p_result[i] == '_')
        {
            break;
        }
        else if(p_object[i] == p_result[i])
        {
            validation = true;
        }
        else
        {
            validation = false;
            break;
        }
    }

    return validation;

}

float calculatePercentage(std::vector<bool> p_vector)
{
    int compteurTrue = 0;
    for(int i = 0; i < p_vector.size(); i++)
    {
        if(p_vector[i])
        {
            compteurTrue++;
        }
    }
    float percentage = ((float)compteurTrue/p_vector.size());
    percentage = percentage*100;
    return percentage;
}

std::vector<double> calculatePercentage(std::vector<int> in_vector)
{
    std::vector<double> results;

    double good_recognition_counter = 0; // 1
    double bad_recognition_counter = 0;  // 0
    double not_recognized_counter = 0;   // -1

    for(int i = 0; i < in_vector.size(); i++)
    {
        if(in_vector.at(i) == 1){
            good_recognition_counter++;
        }
        else if(in_vector.at(i) == 0){
            bad_recognition_counter++;
        }
        else{
            not_recognized_counter++;
        }
    }
    double recognition_percentage = good_recognition_counter/(good_recognition_counter + bad_recognition_counter);
    recognition_percentage = recognition_percentage*100;

    double unrecognized_percentage = not_recognized_counter / (in_vector.size());
    unrecognized_percentage = unrecognized_percentage * 100;

    results.push_back(recognition_percentage);
    results.push_back(unrecognized_percentage);

    return results;
}

boost::filesystem3::path getPointCloudPathFromSignaturePath(boost::filesystem3::path base_pc_path, boost::filesystem3::path sig_filename){
    std::string path;

    boost::filesystem3::path filename = sig_filename.leaf();
    sig_filename.remove_leaf();
    boost::filesystem3::path file_repository = sig_filename.leaf();
    path = base_pc_path.string() + file_repository.string() + "/" + filename.string();

    boost::filesystem3::path new_path(path);
    return new_path;
}

int getPointCloudIndexFromHistogramIndex(int histogram_index, std::vector<int> *surfacePerObject){
    int low = 0;
    int high = 0;

    for(int i=0; i < surfacePerObject->size(); i++){
        high += surfacePerObject->at(i);
        if(histogram_index >= low || histogram_index < high){return i;}
        low = high;
    }

}

void oneLoop(const std::string& p_cvfhSignature, const std::string& p_object, int p_thresold)
{
    std::stringstream ss;
    ss << p_object << "/log_" << p_thresold << ".txt";
    std::string saveFile = ss.str();
    std::ofstream ofs(saveFile.c_str());

    std::vector<int> fine_results;
    std::vector<int> coarse_results;
    std::vector<double> fine_results_time;
    std::vector<double> coarse_results_time;

    boost::filesystem3::path pathObject(p_object);
    boost::filesystem3::recursive_directory_iterator dirItObject(pathObject);

    // Loop for 1 object
    while(dirItObject != boost::filesystem3::recursive_directory_iterator())
    {
        boost::filesystem3::path pathObjectTemp;
        pathObjectTemp = * dirItObject;

        if(pathObjectTemp.extension() == ".pcd" and ! boost::filesystem3::is_directory(pathObjectTemp))
        {
            //boost::filesystem3::path bdPath(BDPATH);
            //boost::filesystem3::recursive_directory_iterator dirItdb(bdPath);

            //boost::filesystem3::path fullScanPath(FULLSCANPATH);
            //boost::filesystem3::directory_iterator dirItScan(fullScanPath);

            boost::filesystem3::path signatureBd(p_cvfhSignature);
            boost::filesystem3::recursive_directory_iterator dirItSignature(signatureBd);


            std::vector<std::string> nameVector;
            std::vector<int> signaturesPerObject;
            pcl::PointCloud<pcl::VFHSignature308>::Ptr signatureCloud_ptr(new pcl::PointCloud<pcl::VFHSignature308>);
            std::vector<boost::filesystem3::path> fullCloud_paths;

            boost::filesystem3::path pathTemp;

            //populate the signature cloud with all the signatures, except it's own
            while(dirItSignature != boost::filesystem3::recursive_directory_iterator())
            {
                pathTemp  = * dirItSignature;
                if(!boost::filesystem3::is_directory(pathTemp) and pathTemp.extension() == ".pcd" && pathTemp.filename() != pathObjectTemp.filename())
                {
                    std::cout << "Loading PointClouds" << std::endl;

                    // Load signatures
                    pcl::PointCloud<pcl::VFHSignature308> tempSig;
                    pcl::io::loadPCDFile(pathTemp.c_str(), tempSig);

                    for(int i = 0; i < tempSig.size(); i ++)
                    {
                        signatureCloud_ptr->push_back(tempSig.at(i));
                        nameVector.push_back(pathTemp.stem().c_str());
                    }

                    //Load related pointcloud paths
                    boost::filesystem3::path pc_path = getPointCloudPathFromSignaturePath(pathObject,pathTemp);

                    // Push the names of pointclouds and how many surfaces per object was found by our-cvfh
                    fullCloud_paths.push_back(pc_path);
                    signaturesPerObject.push_back(tempSig.size());

                }

                dirItSignature++;
            }

            Object_recognition objectRecon;


            // Load cloud and signature we try to recognize
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(pathObjectTemp.c_str(), *loadedCloud);

            pcl::PointCloud<pcl::VFHSignature308>::Ptr signatureLoaded_ptr(new pcl::PointCloud<pcl::VFHSignature308>);
            signatureLoaded_ptr = objectRecon.makeCVFH(loadedCloud);

            std::cout << "Finding Hypotheses for Recognition" << std::endl;

            // *Recognition
            std::vector<float> distances;
            std::vector<std::vector<int> > NN_object_indices = objectRecon.getNNSurfaces(signatureLoaded_ptr,signatureCloud_ptr,5,distances);
            std::vector<pcl::PointCloud<PointT>::Ptr> pc_hypotheses;
            for(int i=0; i < NN_object_indices.at(1).size(); i++){
                int pc_index = getPointCloudIndexFromHistogramIndex(NN_object_indices.at(1).at(i),&signaturesPerObject);
                pcl::PointCloud<PointT>::Ptr tempCloud(new pcl::PointCloud<PointT>);
                std::string path = fullCloud_paths.at(pc_index).string();
                pcl::io::loadPCDFile(path.c_str(), *tempCloud);
                pc_hypotheses.push_back(tempCloud);
            }

            std::cout << "Starting Recognition" << std::endl;
            std::vector<double> results = objectRecon.OURCVFHRecognition(loadedCloud,pc_hypotheses);
            int fine_index = results.at(0);
            int coarse_index = results.at(1);
            double time_fine = results.at(2);
            double time_coarse = results.at(3);

            // Push time for stats at the end
            fine_results_time.push_back(time_fine);
            coarse_results_time.push_back(time_coarse);

            // Write in log file (ofs) and push some data to compile stats at the end

            ofs << "FINE RECOGNITION (with full pointcloud)" << std::endl;
            ofs << "Surface test :  " << pathObjectTemp.stem().c_str() << std::endl;
            ofs << "ICP Time (s) :  " << time_fine<< std::endl;
            std::cout << "FINE RECOGNITION (with full pointcloud)" << std::endl;
            std::cout << "Surface test :  " << pathObjectTemp.stem().c_str() << std::endl;
            std::cout << "ICP Time (s) :  " << time_fine<< std::endl;

            if(fine_index >=0){
                int histogram_index_fine = NN_object_indices.at(1).at(fine_index);
                ofs << "Best match :  " << nameVector.at(histogram_index_fine) << std::endl;
                ofs << "Best match :  " << nameVector.at(histogram_index_fine) << std::endl;
                bool good_recognition = compareName(pathObjectTemp.stem().c_str(), nameVector.at(histogram_index_fine));
                if(good_recognition){
                    fine_results.push_back(1);
                    ofs << "Good recognition :  " <<  "Yes" << std::endl;
                    std::cout << "Good recognition :  " <<  "Yes" << std::endl;
                }
                else{
                    fine_results.push_back(0);
                    ofs << "Good recognition :  " <<  "No" << std::endl;
                    std::cout << "Good recognition :  " <<  "No" << std::endl;
                }
            }
            else{
                fine_results.push_back(-1);
                ofs << "Good recognition :  " <<  "Unable to find a good match" << std::endl;
                std::cout << "Good recognition :  " <<  "Unable to find a good match" << std::endl;
            }



            ofs << std::endl;
            ofs << "COARSE RECOGNITION (with sampled pointcloud)" << std::endl;
            ofs << "Surface test :  " << pathObjectTemp.stem().c_str() << std::endl;
            ofs << "ICP Time (s) :  " << time_coarse<< std::endl;

            std::cout << std::endl;
            std::cout << "COARSE RECOGNITION (with sampled pointcloud)" << std::endl;
            std::cout << "Surface test :  " << pathObjectTemp.stem().c_str() << std::endl;
            std::cout << "ICP Time (s) :  " << time_coarse<< std::endl;

            if(coarse_index >= 0){
                int histogram_index_coarse = NN_object_indices.at(1).at(coarse_index);
                ofs << "Best match :  " << nameVector.at(histogram_index_coarse) << std::endl;
                std::cout << "Best match :  " << nameVector.at(histogram_index_coarse) << std::endl;
                // std::cout << "The object is -> " << nameVector.at(histogram_index_coarse) << "for the coarse transform" << std::endl;
                bool good_recognition = compareName(pathObjectTemp.stem().c_str(), nameVector.at(histogram_index_coarse));
                if(good_recognition){
                    coarse_results.push_back(1);
                    ofs << "Good recognition :  " <<  "Yes" << std::endl;
                    std::cout << "Good recognition :  " <<  "Yes" << std::endl;
                }
                else{
                    coarse_results.push_back(0);
                    ofs << "Good recognition :  " <<  "No" << std::endl;
                    std::cout << "Good recognition :  " <<  "No" << std::endl;
                }
            }

            else{
                coarse_results.push_back(-1);
                ofs << "Good recognition :  " <<  "Unable to find a good match" << std::endl;
                std::cout << "Good recognition :  " <<  "Unable to find a good match" << std::endl;
            }

            ofs << "------------------------" << std::endl;
            std::cout << "------------------------" << std::endl;
        }
        dirItObject++;
    }

    std::vector<double> fineResults = calculatePercentage(fine_results);
    std::vector<double> coarseResults = calculatePercentage(coarse_results);
    //float percentage = calculatePercentage(confirmationVector);
    std::cout << "The fine recognition percentage is " << fineResults.at(0) << "%" << std::endl;
    std::cout << "The percentage of unrecognized objects with fine recognition is " << fineResults.at(1) << "%" << std::endl;
    std::cout << "The coarse recognition percentage is " << coarseResults.at(0) << "%" << std::endl;
    std::cout << "The percentage of unrecognized objects with coarse recognition is " << coarseResults.at(1) << "%" << std::endl;

    ofs << "The fine recognition percentage is " << fineResults.at(0) << "%" << std::endl;
    ofs << "The percentage of unrecognized objects with fine recognition is " << fineResults.at(1) << "%" << std::endl;
    ofs << "The coarse recognition percentage is " << coarseResults.at(0) << "%" << std::endl;
    ofs << "The percentage of unrecognized objects with coarse recognition is " << coarseResults.at(1) << "%" << std::endl;
    ofs.close();
}



int main(int argc, char** argv)
{

    //std::string BDPATH = argv[1];
    //std::string FULLSCANPATH = argv[2];
    std::string CVFHSIGNATUREPATH = argv[1];
    std::string OBJECTLOAD = argv[2];
    std::vector<int> vectorThresold;
    vectorThresold.push_back(1);
//    vectorThresold.push_back(20000);
//    vectorThresold.push_back(50000);
//    vectorThresold.push_back(100000);
//    vectorThresold.push_back(500000);
//    vectorThresold.push_back(1000000);
    for(int i = 0; i < vectorThresold.size(); i++)
    {
        oneLoop(CVFHSIGNATUREPATH, OBJECTLOAD, vectorThresold[i]);
    }
    return 0;
}
