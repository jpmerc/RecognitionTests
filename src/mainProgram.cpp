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

boost::filesystem3::path getPointCloudPathFromSignaturePath(boost::filesystem3::path base_pc_path, boost::filesystem3::path sig_filename){
    boost::filesystem3::path new_path = base_pc_path.parent_path() + sig_filename.filename();
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

    std::vector<bool> confirmationVector;

    boost::filesystem3::path pathObject(p_object);
    boost::filesystem3::recursive_directory_iterator dirItObject(pathObject);

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
            std::vector<boost::filesystem3::path> fullCloud_vector_paths;

            boost::filesystem3::path pathTemp;
            //populate the signature cloud
            while(dirItSignature != boost::filesystem3::recursive_directory_iterator())
            {
                pathTemp  = * dirItSignature;
                if(!boost::filesystem3::is_directory(pathTemp) and pathTemp.extension() == ".pcd" && pathTemp.filename() != pathObjectTemp.filename())
                {
                    // Load signatures
                    pcl::PointCloud<pcl::VFHSignature308> tempSig;
                    pcl::io::loadPCDFile(pathTemp.c_str(), tempSig);
                    signaturesPerObject.push_back(tempSig.size());
                    for(int i = 0; i < tempSig.size(); i ++)
                    {
                        signatureCloud_ptr->push_back(tempSig.at(i));
                        nameVector.push_back(pathTemp.stem().c_str());
                    }

                    //Load related pointcloud paths

                    boost::filesystem3::path pc_path = getPointCloudPathFromSignaturePath(pathObject,pathTemp);
                    //
                    fullCloud_vector_paths.push_back(pc_path);

                }

                dirItSignature++;
            }

            Object_recognition objectRecon;


            // Load cloud and signature we try to recognize
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(pathObjectTemp.c_str(), *loadedCloud);

            pcl::PointCloud<pcl::VFHSignature308>::Ptr signatureLoaded_ptr(new pcl::PointCloud<pcl::VFHSignature308>);
            signatureLoaded_ptr = objectRecon.makeCVFH(loadedCloud);

            // *Recognition
            std::vector<std::vector<int> > NN_object_indices = objectRecon.getNNSurfaces(signatureLoaded_ptr,signatureCloud_ptr);
            std::vector<pcl::PointCloud<PointT>::Ptr> pc_hypotheses;
            for(int i=0; i < NN_object_indices.at(1).size(); i++){
                int pc_index = getPointCloudIndexFromHistogramIndex(NN_object_indices.at(1).at(i),&signaturesPerObject);
                pcl::PointCloud<PointT>::Ptr tempCloud(new pcl::PointCloud<PointT>);
                pcl::io::loadPCDFile(fullCloud_vector_paths.at(pc_index).c_str(), tempCloud);
                pc_hypotheses.push_back(tempCloud);
            }

            std::vector<double> results = objectRecon.OURCVFHRecognition(loadedCloud,pc_hypotheses);
            int fine_index = results.at(0);
            int coarse_index = results.at(1);

            if(fine_index >=0){
                int histogram_index_fine = NN_object_indices.at(1).at(fine_index);
                double time_fine = results.at(2);
                std::cout << "The object is -> " << nameVector.at(histogram_index_fine) << "for the fine transform" << std::endl;

                //Complete to gather the results

            }
            else{
                //Count the number of unfound object and output a stats
            }


            if(coarse_index >= 0){
                int histogram_index_coarse = NN_object_indices.at(1).at(coarse_index);
                double time_coarse = results.at(3);
                std::cout << "The object is -> " << nameVector.at(histogram_index_coarse) << "for the coarse transform" << std::endl;
            }

            else{



            }







            //std::vector<float> vectorResult;
            //vectorResult = objectRecon.histogramComparisonVector(signatureLoaded_ptr,signatureCloud_ptr);



            bool confir = compareName(pathObjectTemp.stem().c_str(), nameVector.at(histogram_index_fine));

            float score = vectorResult[1];

            if(confir and score < p_thresold)
            {
                confirmationVector.push_back(true);
            }
            else
            {
                confirmationVector.push_back(false);
            }
            ofs << "Surface test :  " << pathObjectTemp.stem().c_str() << std::endl;
            ofs << "Best mathc :  " << nameVector.at(vectorResult[0]) << std::endl;
            ofs << "Distance :  " << vectorResult[1] << std::endl;
            ofs << "Confirmation :  " <<  confir << std::endl;
            ofs << "------------------------" << std::endl;
        }
        dirItObject++;
    }
    float percentage = calculatePercentage(confirmationVector);
    std::cout << "The percentage is " << percentage << "%" << std::endl;
    ofs << "The recognition percentage is : " << percentage << "%" << std::endl;
    ofs.close();
}



int main(int argc, char** argv)
{

    //std::string BDPATH = argv[1];
    //std::string FULLSCANPATH = argv[2];
    std::string CVFHSIGNATUREPATH = argv[1];
    std::string OBJECTLOAD = argv[2];
    std::vector<int> vectorThresold;
    vectorThresold.push_back(10000);
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
