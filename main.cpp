#include "cluster.h"
#include "file_read.h"
#include "utils.h"
#include "mapping.h"


int main()
{
	std::cout<<"--------------- EUCLIDEAN CLUSTERINT --------------"<<std::endl;
	file::FileRead file_read;

        file_read.openFile( "test_data5" );

	cluster::Cluster<float> cluster;

	mapping::Mapping<float> mapping;

	cv::Mat image = cv::Mat::zeros( 600, 600, CV_8UC3 );

	int count = 0;
	while( !file_read.endOfFile() ){
		sensor::RecordData data;
	        file_read.readAFrame( data );

		sensor::ScanContainer<float> scan_container;
		Utils::lidarData2DataContainer( data.scan_data, scan_container );		

		Utils::displayAScan( scan_container, image );
	
		std::vector<std::vector<typename sensor::ScanContainer<float>::type>> clusters;
		int num = cluster.extractEuclideanClusters( scan_container, clusters );
		std::cout<<"cluster number = "<< num<<std::endl;

		

		// mapping
		Eigen::Vector3f robot_pose( data.x, data.y, data.theta );
		std::cout<<"robot pose = "<<robot_pose.transpose()<<std::endl;
		
		obstacle::Obstacles<float> obstacles;
		for( auto& cluster : clusters ){
			obstacle::Obstacle<float> obs;
			mapping.getObstacleFromCluster( cluster, obs );
			obstacles.addObstacle( obs );
		}

		if( count % 5 == 0 )
			mapping.updateMap( obstacles, robot_pose );
	//	mapping.updateMap( scan_container, robot_pose );
	//	mapping.displayMap();		

		cv::waitKey(0);
	
		image = cv::Mat::zeros( 600, 600, CV_8UC3 );	
		count ++;
	}
	
		
	file_read.closeFile();

	return 0;
}
