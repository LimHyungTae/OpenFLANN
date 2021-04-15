//
// Created by Hyungtae Lim on 21. 4. 14..
//

#ifndef KDTREE_FLANN_PICOFLANN_PCL_H
#define KDTREE_FLANN_PICOFLANN_PCL_H

#include "picoflann.h"

namespace picoflann_pcl
{

// Adapter class to give to nanoflann the same "look and fell" of pcl::KdTreeFLANN.
// limited to squared distance between 3D points
    template <typename PointT>
    class KdTreeFLANN
    {
    public:

        typedef boost::shared_ptr<KdTreeFLANN<PointT> > Ptr;
        typedef boost::shared_ptr<const KdTreeFLANN<PointT> > ConstPtr;

        typedef typename pcl::PointCloud<PointT> PointCloud;
        typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
        typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        KdTreeFLANN();


        inline Ptr makeShared () { return Ptr (new KdTreeFLANN<PointT> (*this)); }

        void setInputCloud(KdTreeFLANN::PointCloudPtr cloud);

        int  nearestKSearch (const PointT& point, int k, std::vector<int> &k_indices,
                             std::vector<float> &k_sqr_distances);

        int radiusSearch (const PointT &point, double radius, std::vector<int> &k_indices,
                          std::vector<float> &k_sqr_distances) const;

    private:
        struct Point_Adaptor{
            inline   float operator( )(const PointT &elem, int dim)const{
                if (dim ==0)  return elem.x;
                else if (dim == 1)  return elem.y;
                else if (dim == 2)  return elem.z;
                else throw std::invalid_argument("Invalid dimension is coming");
            }
        };

        struct PointCloud_Container{
            size_t _size;
            PointCloudPtr _array;
            PointCloud_Container(){}
            PointCloud_Container(PointCloudPtr& array, size_t Size) {
                _array = array;
                _size = Size;
            }
            inline size_t size()const{return _size;}
            inline const PointT &at(int idx)const{ return _array->points[idx];}
        };
        PointCloud_Container _container;
        picoflann::KdTreeIndex<3, Point_Adaptor> _kdtree; // 3 indicates the dimension: x, y, z
    };

//---------- Definitions ---------------------
    template<typename PointT> inline
    KdTreeFLANN<PointT>::KdTreeFLANN(){}

    template<typename PointT> inline
    void KdTreeFLANN<PointT>::setInputCloud(KdTreeFLANN::PointCloudPtr cloud)
    {
        size_t num_pts = cloud->points.size();
        _kdtree.build(KdTreeFLANN::PointCloud_Container(cloud, num_pts));
        _container = KdTreeFLANN::PointCloud_Container(cloud, num_pts);
    }

    template<typename PointT> inline
    int KdTreeFLANN<PointT>::nearestKSearch(const PointT& point, int num_closest,
                                            std::vector<int> &k_indices,
                                            std::vector<float> &k_sqr_distances)
    {
        std::vector<std::pair<uint32_t,double> > resultSet = _kdtree.searchKnn(_container, point, num_closest);
        const size_t nFound = resultSet.size();
        // Set results in PCL format
        k_indices.resize(nFound);
        k_sqr_distances.resize(nFound);
        for(int i=0; i<nFound; i++ ){
            k_indices[i]       = static_cast<int>(resultSet[i].first);
            k_sqr_distances[i] = static_cast<float>(resultSet[i].second);
        }
        return nFound;
    }

    template<typename PointT> inline
    int KdTreeFLANN<PointT>::radiusSearch(const PointT &point, double radius,
                                          std::vector<int> &k_indices,
                                          std::vector<float> &k_sqr_distances) const
    {
        std::vector<std::pair<uint32_t, double> > resultSet = _kdtree.radiusSearch(_container, point, radius);
        const size_t nFound = resultSet.size();

        k_indices.resize(nFound);
        k_sqr_distances.resize(nFound);
        for(int i=0; i<nFound; i++ ){
            k_indices[i]       = static_cast<int>(resultSet[i].first);
            k_sqr_distances[i] = static_cast<float>(resultSet[i].second);
        }
        return nFound;
    }
}

#endif //KDTREE_FLANN_PICOFLANN_PCL_H
