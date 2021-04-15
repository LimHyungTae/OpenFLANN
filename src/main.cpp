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
    vector<int> num_K_pts = {1, 10, 100, 500, 1000, 5000};
    vector<int> num_radius = {1, 10, 20, 40, 80};

    vector<int> params;
    string target = "radius"; // radius or knn
    if (target == "radius")     params = num_radius;
    else if (target == "knn")   params = num_K_pts;
    for (const int& NUM_RANDOM_SAMPLE_POINTS: num_random_pts) {
        for (const int &parameter: params) {
            cout<<parameter<<" | "<<NUM_RANDOM_SAMPLE_POINTS<<endl;
            string absDir = "/home/shapelim/CLionProjects/kdtree_flann/outputs";
            string txtname = to_string(parameter) + "_" + to_string(NUM_RANDOM_SAMPLE_POINTS) + ".txt";
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
                int K = parameter;

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
                    c_k0 = clock();
                    PCLFLANN_kdtree.radiusSearch(searchPoint, K, idxPcl, distPcl);
                    c_k1 = clock();
                    NanoFLANN_kdtree.radiusSearch(searchPoint, K, idxNano, distNano);
                    c_k2 = clock();
                    PicoFLANN_kdtree.radiusSearch(searchPoint, K, idxPico, distPico);
                    c_k3 = clock();
                }

                TimeSet K_ts = print_time(c_k0, c_k1, c_k2, c_k3);

                pclO << init_ts._t_pcl << " " << K_ts._t_pcl << "\n" << endl;
                nanoO << init_ts._t_nano << " " << K_ts._t_nano << "\n" << endl;
                picoO << init_ts._t_pico << " " << K_ts._t_pico << "\n" << endl;
            }
            pclO.close();
            nanoO.close();
            picoO.close();
        }
    }
    return 0;
}
