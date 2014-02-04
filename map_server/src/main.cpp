/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include <map_server/LoadMap.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class MapServer
{
  public:
    MapServer() {
      ros::NodeHandle private_nh("~");

      private_nh.param("frame_id", frame_id, std::string("map"));

      // Service for loading a new map
      load_service = n.advertiseService("load_map", &MapServer::loadMapCallback, this);
      // Service for requesting map
      service = n.advertiseService("static_map", &MapServer::mapCallback, this);
      // Latched publisher for metadata
      metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      // Latched publisher for data
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    }

    /** Trivial constructor */
    void loadFromFile(const std::string& fname)
    {
      std::string mapfname = "";   
      double origin[3];
      int negate;
      double occ_th, free_th, res;
      bool trinary = true;
      //mapfname = fname + ".pgm";
      //std::ifstream fin((fname + ".yaml").c_str());
      std::ifstream fin(fname.c_str());
      if (fin.fail()) {
        ROS_ERROR("Map_server could not open %s.", fname.c_str());
        exit(-1);
      }
#ifdef HAVE_NEW_YAMLCPP
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
#else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
#endif
      try { 
        doc["resolution"] >> res; 
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        exit(-1);
      }
      try { 
        doc["negate"] >> negate; 
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        exit(-1);
      }
      try { 
        doc["occupied_thresh"] >> occ_th; 
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
        exit(-1);
      }
      try { 
        doc["free_thresh"] >> free_th; 
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
        exit(-1);
      }
      try { 
        doc["trinary"] >> trinary; 
      } catch (YAML::Exception) { 
        ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
        trinary = true;
      }
      try { 
        doc["origin"][0] >> origin[0]; 
        doc["origin"][1] >> origin[1]; 
        doc["origin"][2] >> origin[2]; 
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        exit(-1);
      }
      try { 
        doc["image"] >> mapfname; 
        // TODO: make this path-handling more robust
        if(mapfname.size() == 0)
        {
          ROS_ERROR("The image tag cannot be an empty string.");
          exit(-1);
        }
        if(mapfname[0] != '/')
        {
          // dirname can modify what you pass it
          char* fname_copy = strdup(fname.c_str());
          mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
          free(fname_copy);
        }
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        exit(-1);
      }

      ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
      map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, trinary);
      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;

      // Publish new map
      metadata_pub.publish( meta_data_message_ );
      map_pub.publish( map_resp_.map );
    }

  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::ServiceServer service;
    ros::ServiceServer load_service;
    std::string frame_id;
    bool deprecated;

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res ) {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");

      return true;
    }

    /** Callback invoked when someone wants to load a new service */
    bool loadMapCallback(map_server::LoadMap::Request  &req,
                     map_server::LoadMap::Response &res )
    {
      ROS_INFO("Got map!");

      this->loadFromFile(req.file_name);

      return true;
    }

    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  if(argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  std::string fname(argv[1]);

  try
  {
    MapServer ms;
    ms.loadFromFile(fname);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}

