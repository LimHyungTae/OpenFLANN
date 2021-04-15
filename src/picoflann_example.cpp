//
// Created by shapelim on 21. 4. 14..
//

#include <random>
#include <vector>
#include <fstream>
#include "picoflann.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <exception>
void example1(){
    //Data type
    struct Point2f{
        Point2f(float X,float Y) { x=X;y=Y; }
        float x,y;
    };

    // Adapter.
    // Given an Point2f element, it returns the element of the dimension specified such that dim=0 is x and dim=1 is y
    struct PicoFlann_Point2fAdapter{
        inline  float operator( )(const Point2f &elem, int dim)const { return dim==0?elem.x:elem.y; }
    };

    //create the points randomly
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-1000.0,1000.0);
    std::vector<Point2f> data;
    for(size_t i=0;i<1000;i++)
        data.push_back( Point2f ( distribution(generator),distribution(generator)));
    ///------------------------------------------------------------
    /// Create the kdtree
    picoflann::KdTreeIndex<2,PicoFlann_Point2fAdapter>  kdtree;//2 is the number of dimensions
    kdtree.build(data);
    //search 10 nearest neibors to point (0,0)
    std::vector<std::pair<uint32_t,double> > res=kdtree.searchKnn(data,Point2f(0,0),10);

    //radius search in a radius of 30 (the resulting distances are squared)
    res=kdtree.radiusSearch(data,Point2f(0,0),30);
    //another version
    kdtree.radiusSearch(res,data,Point2f(0,0),30);

    //you can save to a file
    std::ofstream file_out("out.bin",std::ios::binary);
    kdtree.toStream(file_out);

    //recover from the file
    picoflann::KdTreeIndex<2,PicoFlann_Point2fAdapter>  kdtree2;
    std::ifstream file_in("out.bin",std::ios::binary);
    kdtree2.fromStream(file_in);
    res=kdtree2.radiusSearch(data,Point2f(0,0),30);

}


//Using an array of 3d points
void example2(){

    struct Point3f{
        Point3f(float X,float Y,float Z) { data[0]=X;data[1]=Y;data[2]=Z; }
        float data[3];
    };
    struct PicoFlann_Array3f_Adapter{
        inline   float operator( )(const Point3f &elem, int dim)const{ return elem.data[dim]; }
    };
    struct PicoFlann_Array3f_Container{
        const Point3f *_array;
        size_t _size;
        PicoFlann_Array3f_Container(float *array,size_t Size):_array((Point3f*)array),_size(Size){}
        inline size_t size()const{return _size;}
        inline const Point3f &at(int idx)const{ return _array [idx];}
    };
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-1000.0,1000.0);

    int nPoints=1000;
    float *array=new float[nPoints*3];
    for(size_t i=0;i<1000*3;i++)
        array[i]= distribution(generator);

    ///------------------------------------------------------------
    picoflann::KdTreeIndex<3, PicoFlann_Array3f_Adapter> kdtree;// 3 is the number of dimensions, L2 is the type of distance
    kdtree.build( PicoFlann_Array3f_Container(array, nPoints));
    PicoFlann_Array3f_Container p3container(array,nPoints);
    std::vector<std::pair<uint32_t,double> > res=kdtree.searchKnn(p3container,Point3f(0,0,0),10);
    res=kdtree.radiusSearch(p3container,Point3f(0,0,0),30);
    std::cout<<"Done!"<<std::endl;
}

//Using an array of 3d points
void example3(){

    struct Point3f{
        Point3f(float X,float Y,float Z) { data[0]=X;data[1]=Y;data[2]=Z; }
        float data[3];
    };

    struct PicoFlann_PCL_Point_Adapter{
        inline   float operator( )(const pcl::PointXYZ &elem, int dim)const{
            if (dim ==0)  return elem.x;
            else if (dim == 1)  return elem.y;
            else if (dim == 2)  return elem.z;
            else throw std::invalid_argument("Invalid dimension is comming");
        }
    };
    struct PicoFlann_PCL_Cloud_Container{
        size_t _size;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _array;

        PicoFlann_PCL_Cloud_Container(pcl::PointCloud<pcl::PointXYZ>::Ptr& array, size_t Size) {
           _array = array;
           _size = Size;
        }
        inline size_t size()const{return _size;}
        inline const pcl::PointXYZ &at(int idx)const{ return _array->points[idx];}
    };
    int NUM_RANDOM_SAMPLE_POINTS = 1000;

    double RANDOM_SCALE = 100.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Generate pointcloud data
    cloud->width = NUM_RANDOM_SAMPLE_POINTS;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size (); ++i)
    {
        (*cloud)[i].x = RANDOM_SCALE * rand () / (RAND_MAX + 1.0f);
        (*cloud)[i].y = RANDOM_SCALE * rand () / (RAND_MAX + 1.0f);
        (*cloud)[i].z = RANDOM_SCALE * rand () / (RAND_MAX + 1.0f);
    }
    ///------------------------------------------------------------
    pcl::PointXYZ searchPoint;

    searchPoint.x = RANDOM_SCALE * rand () / (RAND_MAX + 1.0f);
    searchPoint.y = RANDOM_SCALE * rand () / (RAND_MAX + 1.0f);
    searchPoint.z = RANDOM_SCALE * rand () / (RAND_MAX + 1.0f);

    picoflann::KdTreeIndex<3, PicoFlann_PCL_Point_Adapter> kdtree;// 3 is the number of dimensions, L2 is the type of distance
    kdtree.build( PicoFlann_PCL_Cloud_Container(cloud, NUM_RANDOM_SAMPLE_POINTS));
    PicoFlann_PCL_Cloud_Container p3container(cloud, NUM_RANDOM_SAMPLE_POINTS);
    std::vector<std::pair<uint32_t,double> > res=kdtree.searchKnn(p3container,searchPoint,10);
    res=kdtree.radiusSearch(p3container,searchPoint,30);
    std::cout<<"Done!"<<std::endl;
}
int main(){
    example3();
    return 0;
}