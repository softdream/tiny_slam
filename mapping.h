#ifndef __MAPPING_H
#define __MAPPING_H

#include <Eigen/Dense>
#include "obstacles.h"

#define DISPLAY

#ifdef DISPLAY
#include <opencv2/opencv.hpp>
#endif

namespace mapping
{

template<typename T>
class Mapping
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;
	using Vector3 = typename Eigen::Matrix<DataType, 3, 1>;

	Mapping()
	{
		map.resize( map_width, std::vector<uint8_t>( map_height, 0 ) );
	}

	Mapping( const int map_width_, const int map_height_ ) : map_width(map_width_), map_height(map_height)
        {
                map.resize( map_width, std::vector<uint8_t>( map_height, 0 ) );


        }


	~Mapping()
	{

	}

	void updateMap( const sensor::ScanContainer<DataType>& scan_container, const Vector3& robot_world )
	{
		for( int i = 0; i < scan_container.getSize(); i ++ ){
			typename sensor::ScanContainer<DataType>::type pt_laser = scan_container.getIndexData( i );
			Vector2 pt_world = pointLaser2World( pt_laser, robot_world );
			//std::cout<<"world["<<i<<"] = "<<pt_world.transpose()<<std::endl; 
			Vector2 pt_map = pt_world * scale + Vector2( static_cast<DataType>( map_width ) * 0.5, static_cast<DataType>( map_height ) * 0.5 );
			//std::cout<<"pt_map["<<i<<"] = "<<pt_map.transpose()<<std::endl;
			
			//Eigen::Vector2i pt_map_inter = pt_map.cast<int>();
			Eigen::Vector2i pt_map_inter( static_cast<int>( pt_map[0] ), static_cast<int>( pt_map[1] ) );		
	
			if( !isOutOfRange( pt_map ) ){
				map[pt_map_inter[0]][pt_map_inter[1]] = 1;	
			}
			else { // need resize the map

			}
		}	
	}

	void updateMap( const obstacle::Obstacles<DataType>& obstacles, const Vector3& robot_world )
        {
	//	cv::Mat image = cv::Mat::zeros( map_width, map_height, CV_8UC3 );
		Vector2 robot_world2( robot_world[0], robot_world[1] );
		Vector2 robot_map = robot_world2 * scale + Vector2( static_cast<DataType>( map_width ) * 0.5, static_cast<DataType>( map_height ) * 0.5 );

		cv::circle(image, cv::Point_<DataType>( robot_map[0], robot_map[1] ), 3, cv::Scalar(0, 255, 255), -1);

		for( int i = 0; i < obstacles.getSize(); i ++ ){
			obstacle::Obstacle<DataType> obs = obstacles.getIndexData( i );
			Vector2 pose_world = pointLaser2World( obs.pose_, robot_world );
			Vector2 pose_map = pose_world * scale + Vector2( static_cast<DataType>( map_width ) * 0.5, static_cast<DataType>( map_height ) * 0.5 );
			
			cv::Point_<DataType> pose_map_cv( pose_map[0], pose_map[1] );			
			cv::circle(image, pose_map_cv, 3, cv::Scalar(0, 0, 255), -1);			

			std::vector<cv::Point_<DataType>> rect_pt_vec( 4 );
			for( int i = 0; i < 4; i ++ ){
				Vector2 pt_world = pointLaser2World( obs.rect_[i], robot_world );
				Vector2 pt_map = pt_world * scale + Vector2( static_cast<DataType>( map_width ) * 0.5, static_cast<DataType>( map_height ) * 0.5 );
			
				rect_pt_vec[i].x = pt_map[0];
				rect_pt_vec[i].y = pt_map[1];
			}

			for (int i = 0; i < 4; i++){
                        	cv::line(image, rect_pt_vec[i], rect_pt_vec[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
                	}
		}
		cv::imshow( "map", image );
		cv::imwrite( std::to_string( count ) + ".png", image );
		count ++;
        }


#ifdef DISPLAY
	void displayMap(  )
	{
		cv::Mat image = cv::Mat::zeros( map_width, map_height, CV_8UC3 );

		for( int i = 0; i < map_width; i ++ ){
			for( int j = 0; j < map_height; j ++ ){
				if( map[i][j] == 1 ){
					cv::circle(image, cv::Point(i, j), 3, cv::Scalar(0, 0, 255), -1);
				}
			}
		}
	
		cv::imshow( "map", image );
	}
	
#endif

	void resizeMap()
	{

	}

	void resetMap()
	{
		return map.clear();
	}

	void getObstacleFromCluster( const std::vector<typename sensor::ScanContainer<DataType>::type>& cluster, obstacle::Obstacle<DataType>& obs )
	{
		Vector2 pt_all = Vector2::Zero();

		std::vector<cv::Point_<DataType>> points;

                for( auto& pt : cluster ){
                        pt_all += pt;
			points.push_back( cv::Point_<DataType>( pt[0], pt[1] ) );
                }
                pt_all /= static_cast<DataType>( cluster.size() );
			
		cv::RotatedRect box = cv::minAreaRect( cv::Mat( points ) );
	
		cv::Point_<DataType> tr[4];
		box.points( tr );
	
		for( int i = 0; i < 4; i ++ ){
			//obs.rect_[i][0] = tr[i][0];
			//obs.rect_[i][1] = tr[i][1];

			obs.rect_[i][0] = tr[i].x;
			obs.rect_[i][1] = tr[i].y;
			std::cout<<"obs.rect_["<<i<<"] = "<<obs.rect_[i].transpose()<<std::endl;
		}	
		obs.pose_ = pt_all;
	
                obs.length_ = ( obs.rect_[0] - obs.rect_[2] ).norm();
                std::cout<<"length = "<<obs.length_<<", pose = "<<obs.pose_.transpose()<<std::endl;
	}

private:
	const Vector2 pointLaser2World( const Vector2& pt_laser, const Vector3& robot_world )
	{
		Eigen::Matrix<DataType, 2, 2> rotate;
                rotate << ::cos( robot_world[2] ), -::sin( robot_world[2] ),
                          ::sin( robot_world[2] ),  ::cos( robot_world[2] );

		Vector2 trans( robot_world[0], robot_world[1] );
	
		return rotate * pt_laser + trans;
	}

	const Vector2 pointWorld2Laser( const Vector2& pt_world, const Vector3& robot_world )
	{
		Eigen::Matrix<DataType, 2, 2> rotate;
                rotate << ::cos( robot_world[2] ), -::sin( robot_world[2] ),
                          ::sin( robot_world[2] ),  ::cos( robot_world[2] );

		Vector2 trans( robot_world[0], robot_world[1] );

		return rotate.inverse() * pt_world - trans;
	}

	const bool isOutOfRange( const DataType x, const DataType y )
	{
		if( x < 0 || x > static_cast<DataType>( map_width ) || y < 0 || y > static_cast<DataType>( map_height ) ){
			return true;
		}

		return false;
	}

	const bool isOutOfRange( const Vector2& pose )
	{
		return isOutOfRange( pose[0], pose[1] );
	}


private:
	int map_width = 800;
	int map_height = 800;
	DataType scale = 500;
	std::vector<std::vector<uint8_t>> map;

#ifdef DISPLAY
	cv::Mat image = cv::Mat::zeros( map_width, map_height, CV_8UC3 );
#endif
	int count = 0;
};


}

#endif
