#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <ctime>
#include "nanoflann_pcl.h"
#include "picoflann_pcl.h"

#define RANDOM_SCALE 100.0

using namespace std;

struct TimeSet{
    float _t_pcl;
    float _t_nano;
    float _t_pico;
    TimeSet(float t_pcl, float t_nano, float t_pico): _t_pcl(t_pcl), _t_nano(t_nano), _t_pico(t_pico){}
};
TimeSet print_time(const clock_t& c0, const clock_t& c1, const clock_t& c2, const clock_t& c3){
    float t_pcl = float(c1 - c0) /  CLOCKS_PER_SEC;
    float t_nano = float(c2 - c1) /  CLOCKS_PER_SEC;
    float t_pico = float(c3 - c2) /  CLOCKS_PER_SEC;

//    cout<<t_pcl<<" | "<<t_nano<<" | "<<t_pico<<endl;
    return TimeSet(t_pcl, t_nano, t_pico);
}
template <typename T>
void print_vectors(vector<T>& v0, vector<T>& v1, vector<T>& v2){
    assert(v0.size() == v1.size());
    assert(v1.size() == v2.size());
    assert(v2.size() == v0.size());
    for (int i=0; i< v0.size(); ++i){
        cout<<v0[i]<<" | "<<v1[i]<< " | "<<v2[i]<<endl;
    }
}

int main (int argc, char** argv)
{
    srand (time (NULL));
    pcl::KdTreeFLANN<pcl::PointXYZ> PCLFLANN_kdtree;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> NanoFLANN_kdtree;
    picoflann_pcl::KdTreeFLANN<pcl::PointXYZ> PicoFLANN_kdtree;

    vector<int> num_random_pts = {10000, 30000, 60000, 100000, 200000};
    vector<int> K_pts = {1, 10, 100, 500, 1000, 5000};
    vector<double> radiuses = {5.0, 10.0, 20.0, 40.0, 80.0};

    // --------- Set target param -----------
    string target = "knn"; // radius or knn
    // --------------------------------------
    int num_params;
    if (target == "radius"){
        num_params = radiuses.size();
    }
    else if (target == "knn"){
        num_params = K_pts.size();
    }

    string absDir = "/home/shapelim/CLionProjects/OpenFLANN/outputs";
    double searchRadius; // for radiusSearch
    int K; // for KNN
    for (const int& NUM_RANDOM_SAMPLE_POINTS: num_random_pts) {
        for (int ii = 0; ii < num_params; ++ii) {
            string targetName;
            if (target == "radius"){
                searchRadius = radiuses[ii];
                targetName = to_string(int(round(searchRadius * 10) / 10));
            }
            else if (target == "knn"){
                K = K_pts[ii];
                targetName = to_string(K);
            }

            string txtname = targetName + "_" + to_string(NUM_RANDOM_SAMPLE_POINTS) + ".txt";
            string pclFilename = absDir + "/pcl_" + txtname;
            string nanoFilename = absDir + "/nano_" + txtname;
            string picoFilename = absDir + "/pico_" + txtname;
            std::ofstream pclO(pclFilename, ios::app);
            std::ofstream nanoO(nanoFilename, ios::app);
            std::ofstream picoO(picoFilename, ios::app);

            for (int dummyIdx = 0; dummyIdx < 1000; ++dummyIdx) {

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

                // Generate pointcloud data
                cloud->width = NUM_RANDOM_SAMPLE_POINTS;
                cloud->height = 1;
                cloud->points.resize(cloud->width * cloud->height);

                for (std::size_t i = 0; i < cloud->size(); ++i) {
                    (*cloud)[i].x = RANDOM_SCALE * rand() / (RAND_MAX + 1.0f);
                    (*cloud)[i].y = RANDOM_SCALE * rand() / (RAND_MAX + 1.0f);
                    (*cloud)[i].z = RANDOM_SCALE * rand() / (RAND_MAX + 1.0f);
                }

                // check time
                clock_t c_i0 = clock();
                PCLFLANN_kdtree.setInputCloud(cloud);
                clock_t c_i1 = clock();
                NanoFLANN_kdtree.setInputCloud(cloud);
                clock_t c_i2 = clock();
                PicoFLANN_kdtree.setInputCloud(cloud);
                clock_t c_i3 = clock();

                TimeSet init_ts = print_time(c_i0, c_i1, c_i2, c_i3);

                pcl::PointXYZ searchPoint;

                searchPoint.x = RANDOM_SCALE * rand() / (RAND_MAX + 1.0f);
                searchPoint.y = RANDOM_SCALE * rand() / (RAND_MAX + 1.0f);
                searchPoint.z = RANDOM_SCALE * rand() / (RAND_MAX + 1.0f);

                // K nearest neighbor search or Radius

                std::vector<int> idxPcl(K), idxNano(K), idxPico(K);
                std::vector<float> distPcl(K), distNano(K), distPico(K);
                clock_t c_k0, c_k1, c_k2, c_k3;
                if (target == "knn"){
                    c_k0 = clock();
                    PCLFLANN_kdtree.nearestKSearch(searchPoint, K, idxPcl, distPcl);
                    c_k1 = clock();
                    NanoFLANN_kdtree.nearestKSearch(searchPoint, K, idxNano, distNano);
                    c_k2 = clock();
                    PicoFLANN_kdtree.nearestKSearch(searchPoint, K, idxPico, distPico);
                    c_k3 = clock();
                }else if (target == "radius"){
                    std::cout<<"Searching radius..."<<std::endl;
                    c_k0 = clock();
                    PCLFLANN_kdtree.radiusSearch(searchPoint, searchRadius, idxPcl, distPcl);
                    c_k1 = clock();
                    NanoFLANN_kdtree.radiusSearch(searchPoint, searchRadius, idxNano, distNano);
                    c_k2 = clock();
                    PicoFLANN_kdtree.radiusSearch(searchPoint, searchRadius, idxPico, distPico);
                    c_k3 = clock();
//                    To check the size!
//                    cout<<idxPcl.size()<<" , "<<idxNano.size()<< " , "<<idxPico.size()<<endl;
//                    cout<<distPcl.size()<<" , "<<distNano.size()<< " , "<<distPico.size()<<endl;
                }

                TimeSet K_ts = print_time(c_k0, c_k1, c_k2, c_k3);

                pclO << init_ts._t_pcl << " " << K_ts._t_pcl<< endl;
                nanoO << init_ts._t_nano << " " << K_ts._t_nano<< endl;
                picoO << init_ts._t_pico << " " << K_ts._t_pico<< endl;
            }
            pclO.close();
            nanoO.close();
            picoO.close();
        }
    }
    return 0;
}
