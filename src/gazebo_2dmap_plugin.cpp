/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_2dmap_plugin.h"

#include <gazebo/common/Time.hh>
#include <gazebo/common/CommonTypes.hh>
//#include <gazebo/math/Vector3.hh>
#include <ignition/math/Vector3.hh>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

namespace gazebo {

OccupancyMapFromWorld::~OccupancyMapFromWorld()
{
  if(occ_grid_rviz_pub_th_.joinable())
    occ_grid_rviz_pub_th_.join();
}

void OccupancyMapFromWorld::Load(physics::WorldPtr _parent,
                                 sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  world_ = _parent;

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  map_service_ = nh_.advertiseService(
        "gazebo_2dmap_plugin/generate_map", &OccupancyMapFromWorld::ServiceCallback, this);

  map_resolution_ = 0.1;

  if(_sdf->HasElement("map_resolution"))
    map_resolution_ = _sdf->GetElement("map_resolution")->Get<double>();

  slice_height_ = 0.3;

  if(_sdf->HasElement("slice_height"))
    slice_height_ = _sdf->GetElement("slice_height")->Get<double>();

  occupancy_map_update_rate_ = 60.0;
  if(_sdf->HasElement("occupancy_map_update_rate"))
    occupancy_map_update_rate_ = _sdf->GetElement("occupancy_map_update_rate")->Get<double>();

  map_origin_ = ignition::math::Vector3d(0, 0, 0);

  if(_sdf->HasElement("map_origin"))
  {
    if(_sdf->GetElement("map_origin")->HasElement("x"))
      map_origin_.X(_sdf->GetElement("map_origin")->GetElement("x")->Get<double>());
    if(_sdf->GetElement("map_origin")->HasElement("y"))
      map_origin_.Y(_sdf->GetElement("map_origin")->GetElement("y")->Get<double>());
    if(_sdf->GetElement("map_origin")->HasElement("z"))
      map_origin_.Z(_sdf->GetElement("map_origin")->GetElement("z")->Get<double>());
  }

  //ROS_WARN_STREAM("X: " << map_origin_.X() << " " << "Y: " << map_origin_.Y() << "Z: " << map_origin_.Z());

  map_size_x_ = 10.0;

  if(_sdf->HasElement("map_size_x"))
    map_size_x_ = _sdf->GetElement("map_size_x")->Get<double>();

  map_size_y_ = 10.0;

  if(_sdf->HasElement("map_size_y"))
    map_size_y_ = _sdf->GetElement("map_size_y")->Get<double>();

  if(_sdf->HasElement("full_file_path"))
    full_file_path_ = _sdf->GetElement("full_file_path")->Get<std::string>();
  
  CreateOccupancyMap();

  occ_grid_rviz_pub_th_ = std::thread(std::bind(&OccupancyMapFromWorld::OccupancyGridToRviz, this));
}

bool OccupancyMapFromWorld::ServiceCallback(std_srvs::Empty::Request& req,
                                            std_srvs::Empty::Response& res)
{
  CreateOccupancyMap();
  return true;
}

bool OccupancyMapFromWorld::worldCellIntersection(const ignition::math::Vector3d& cell_center,
                                                  const double cell_length,
                                                  gazebo::physics::RayShapePtr ray)
{
  //check for collisions with rays surrounding the cell
  //    ---
  //   | + |
  //    ---

  double dist;
  std::string entity_name;

  int cell_length_steps = 10;
  double side_length;

  //check for collisions with beams at increasing sizes to capture smaller
  //objects inside the cell
  for(int step=1; step<=cell_length_steps; step++)
  {
    side_length = cell_length / cell_length_steps * step;

    for(int i=-1; i<2; i+=2)
    {
      double start_x = cell_center.X() + i * side_length/2;
      double start_y = cell_center.Y() - i * side_length/2;

      for(int j=-1; j<2; j+=2)
      {
        double end_x = cell_center.X() + j * side_length/2;
        double end_y = cell_center.Y() + j * side_length/2;

        //      std::cout << "start_x" << start_x << std::endl;
        //      std::cout << "start_y" << start_y << std::endl;
        //      std::cout << "end_x" << end_x << std::endl;
        //      std::cout << "end_y" << end_y << std::endl;

        ray->SetPoints(ignition::math::Vector3d(start_x, start_y, cell_center.Z()),
                       ignition::math::Vector3d(end_x, end_y, cell_center.Z()));
        ray->GetIntersection(dist, entity_name);

        if(!entity_name.empty())
          return true;
      }
    }
  }

  return false;
}

void OccupancyMapFromWorld::cell2world(unsigned int cell_x, unsigned int cell_y,
                                       double map_size_x, double map_size_y,
                                       double map_resolution,
                                       double& world_x, double &world_y)
{
  /*world_x = cell_x * map_resolution - map_size_x/2 + map_resolution/2;
  world_y = cell_y * map_resolution - map_size_y/2 + map_resolution/2;*/
  world_x = cell_x * map_resolution + map_resolution/2 + map_origin_.X();
  world_y = cell_y * map_resolution + map_resolution/2 + map_origin_.Y();
}

void OccupancyMapFromWorld::world2cell(double world_x, double world_y,
                                       double map_size_x, double map_size_y,
                                       double map_resolution,
                                       unsigned int& cell_x, unsigned int& cell_y)
{
  /*cell_x = (world_x + map_size_x/2) / map_resolution;
  cell_y = (world_y + map_size_y/2) / map_resolution;*/
  cell_x = (world_x - map_origin_.X()) / map_resolution;
  cell_y = (world_y - map_origin_.Y()) / map_resolution;
}

bool OccupancyMapFromWorld::cell2index(int cell_x, int cell_y,
                                       unsigned int cell_size_x, unsigned int cell_size_y,
                                       unsigned int& map_index)
{
  if(cell_x >= 0 && cell_x < cell_size_x && cell_y >= 0 && cell_y < cell_size_y)
  {
    map_index = cell_y * cell_size_y + cell_x;
    //map_index = cell_x * cell_size_x + cell_y;
    return true;
  }
  else
  {
    //return false when outside map bounds
    return false;
  }
}

bool OccupancyMapFromWorld::index2cell(int index, unsigned int cell_size_x,
                                       unsigned int cell_size_y,
                                       unsigned int& cell_x, unsigned int& cell_y)
{
  cell_y = index / cell_size_y;
  cell_x = index % cell_size_x;

  if(cell_x >= 0 && cell_x < cell_size_x && cell_y >= 0 && cell_y < cell_size_y)
    return true;
  else
  {
    //return false when outside map bounds
    return false;
  }
}

void grid2image(nav_msgs::OccupancyGrid* map, std::string map_name)
    {
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);

      int threshold_occupied_ = 65;
  	  int threshold_free_ = 25;

      std::string mapdatafile = map_name + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          //unsigned int i = y * map->info.width + x;
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (map->data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else { //occ [0.25,0.65]
            // an inaccessible cell
            //fputc(205, out);
            fputc(000, out);
          }
        }
      }

      fclose(out);

      /*Magick::Image i(mapdatafile);
      auto desired_x = i.rows()/2;
      auto desired_y = i.columns()/2;
      i.crop(Magick::Geometry(0, desired_y));
      i.chop(Magick::Geometry(desired_x, 0));
      i.write(mapdatafile);*/


      std::string mapmetadatafile = map_name + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
       */

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done\n");
    }

void OccupancyMapFromWorld::OccupancyGridToRviz()
{
  ros::Duration update_time(occupancy_map_update_rate_);
  // Don't start world scan until models don't load
  update_time.sleep();
  while(ros::ok())
  {
    CreateOccupancyMap();
    update_time.sleep();
  }

}

void OccupancyMapFromWorld::CreateOccupancyMap()
{
  // Set up the origin based on the map origin specified
  ignition::math::Vector3d map_origin(map_origin_.X(), map_origin_.Y(),
                                      map_origin_.Z() + slice_height_);

  unsigned int cells_size_x = map_size_x_ / map_resolution_;
  unsigned int cells_size_y = map_size_y_ / map_resolution_;

  occupancy_map_ = new nav_msgs::OccupancyGrid();
  occupancy_map_->data.resize(cells_size_x * cells_size_y);
  //all cells are initially unknown
  std::fill(occupancy_map_->data.begin(), occupancy_map_->data.end(), -1);
  occupancy_map_->header.stamp = ros::Time::now();
  occupancy_map_->header.frame_id = "map"; //TODO map frame
  occupancy_map_->info.map_load_time = ros::Time(0);
  occupancy_map_->info.resolution = map_resolution_;
  occupancy_map_->info.width = cells_size_x;
  occupancy_map_->info.height = cells_size_y;
  occupancy_map_->info.origin.position.x = map_origin_.X();
  occupancy_map_->info.origin.position.y = map_origin_.Y();
  occupancy_map_->info.origin.position.z = map_origin_.Z();
  occupancy_map_->info.origin.orientation.w = 1;

  gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  //std::cout << "Starting wavefront expansion for mapping" << std::endl;
  ROS_INFO("Starting wavefront expansion for mapping\n");  

  //identify free space by spreading out from initial robot cell
  double robot_x = 0;
  double robot_y = 0;

  //find initial robot cell
  unsigned int cell_x=0, cell_y=0, map_index;
  world2cell(robot_x, robot_y, map_size_x_, map_size_y_, map_resolution_,
             cell_x, cell_y);

  if(!cell2index(cell_x, cell_y, cells_size_x, cells_size_y, map_index))
  {
    ROS_ERROR_NAMED(name_, "initial robot pos is outside map, could not create "
                           "map");
    return;
  }

  //ROS_INFO_STREAM("CELL_X: " << cell_x << " " << "CELL_Y: " << cell_y);

  //ROS_INFO_STREAM("cells_size_x: " << cells_size_x << " " << "cells_size_y: " << cells_size_y);

  //ROS_INFO_STREAM("cells_size_x: " << map_index);

  std::vector<unsigned int> wavefront;
  wavefront.push_back(map_index);

  //wavefront expansion for identifying free, unknown and occupied cells
  while(!wavefront.empty())
  {
    map_index = wavefront.at(0);
    wavefront.erase(wavefront.begin());

    index2cell(map_index, cells_size_x, cells_size_y, cell_x, cell_y);

    //mark cell as free
    occupancy_map_->data.at(map_index) = 0;

    //explore cells neighbors in an 8-connected grid
    unsigned int child_index;
    double world_x, world_y;
    uint8_t child_val;

    //8-connected grid
    for(int i=-1; i<2; i++)
    {
      for(int j=-1; j<2; j++)
      {
        //makes sure index is inside map bounds
        if(cell2index(cell_x + i, cell_y + j, cells_size_x, cells_size_y, child_index))
        {
          child_val = occupancy_map_->data.at(child_index);

          //only update value if cell is unknown
          if(child_val != 100 && child_val != 0 && child_val != 50)
          {
            cell2world(cell_x + i, cell_y + j, map_size_x_, map_size_y_, map_resolution_,
                       world_x, world_y);

            bool cell_occupied = worldCellIntersection(ignition::math::Vector3d(world_x, world_y, slice_height_),
                                                       map_resolution_, ray);

            if(cell_occupied)
              //mark cell as occupied
              occupancy_map_->data.at(child_index) = 100;


            else
            {
              //add cell to wavefront
              wavefront.push_back(child_index);
              //mark wavefront in map so we don't add children to wavefront multiple
              //times
              // all inaccessible cells are filled in black
              occupancy_map_->data.at(child_index) = 50;
            }
          }
        }
      }
    }
  }

  //std::system("rosrun map_server map_saver -f env_test &");

  map_pub_.publish(*occupancy_map_);
  ROS_INFO("Occupancy Map generation completed\n");

  grid2image(occupancy_map_, full_file_path_);

  //std::cout << "\rOccupancy Map generation completed                  " << std::endl;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OccupancyMapFromWorld)

}  // namespace gazebo
