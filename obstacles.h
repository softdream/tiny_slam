#ifndef __OBSTACLES_H
#define __OBSTACLES_H

#include <Eigen/Dense>
#include <vector>

namespace obstacle
{

template<typename T>
struct Obstacle
{
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;

	Obstacle()
	{

	}

	Obstacle( const Vector2 rect[4], const Vector2& pose ) : rect_{ rect[0], rect[1], rect[2], rect[3] }, pose_(pose)
	{

	}

	~Obstacle()
	{

	}

	Vector2 pose_ = Vector2::Zero();
	Vector2 rect_[4];
};

template<typename T>
class Obstacles
{
public:
	using DataType = T;
        using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;

	Obstacles()
	{

	}

	~Obstacles()
	{

	}

	void addObstacle( const Obstacle<DataType>& obs )
	{
		return obs_vec.push_back( obs );
	}

	void addObstacle( const DataType rect[4], const Vector2& pose )
	{
		return obs_vec.push_back( Obstacle<DataType>( rect, pose ) );
	}

	void clearAll()
	{
		return obs_vec.clear();
	}

	const int getSize() const
	{
		return obs_vec.size();
	}

	const bool isEmpty() const
	{
		return obs_vec.empty();
	}

	const Obstacle<DataType>& operator[]( const int i ) const
	{
		return obs_vec[i];
	}

	const Obstacle<DataType>& getIndexData( const int i ) const
	{
		return obs_vec[i];
	}

private:
	std::vector<Obstacle<DataType>> obs_vec;
};

}


#endif
