/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Dirk Holz, University of Bonn.
 *  All rights reserved.
 *  The code is based on early pcl example code.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
#include <pcl_conversions/pcl_conversions.h>
typedef pcl::PCLPointCloud2 PointCloud2;
#else
typedef sensor_msgs::PointCloud2 PointCloud2;
#endif

typedef pcl::PointXYZ Point;
typedef pcl::visualization::PointCloudColorHandler<PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

typedef PointCloud2::Ptr PointCloudPtr;
typedef PointCloud2::ConstPtr PointCloudConstPtr;
PointCloudConstPtr cloud_, cloud_old_;
boost::mutex m;

bool paused = false;
bool record_continuously = false;
bool record_single = false;
std::string topic_name = "";

bool record_fixed_number = false;
int rec_max_frames = 100;
int rec_nr_frames = 0;

void cloud_cb (const PointCloudConstPtr& cloud)
{
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
  std_msgs::Header header = pcl_conversions::fromPCL(cloud->header);
#else
  std_msgs::Header header = cloud->header;
#endif
  float stamp = header.stamp.toSec ();

  ROS_INFO ("PointCloud with %d data points (%s), stamp %f, and frame %s.", 
            cloud->width * cloud->height, 
            pcl::getFieldsList (*cloud).c_str (), 
            stamp, 
            cloud->header.frame_id.c_str ()); 
  m.lock ();

  cloud_ = cloud;

  if(record_continuously || record_single)
  {
    boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%Y_%m_%d_%H_%M_%s");
    std::basic_stringstream<char> ss;
    ss.imbue(std::locale(std::cout.getloc(), facet));
    ss << header.stamp.toBoost();
    std::string formatted_stamp = ss.str();
    replace(formatted_stamp.begin(), formatted_stamp.end(), '.', '_');

    std::string filename = str(boost::format("%s_%s.pcd") 
                               % topic_name 
                               % formatted_stamp);
    replace(filename.begin(), filename.end(), '/', '_');
    
    try
    {
      pcl::io::savePCDFile(filename.c_str(), 
                           *cloud,
                           Eigen::Vector4f::Zero(), 
                           Eigen::Quaternionf::Identity(), 
                           true);
      if (record_single)
        pcl::console::print_info("Stored file %s.\n", filename.c_str());
      record_single = false;
    }
    catch (pcl::IOException& e)
    {
      std::cerr << e.what() << std::endl;
      return;
    }

    if (record_fixed_number)
      if (++rec_nr_frames >= rec_max_frames)
        record_continuously = false;
  }
  
  m.unlock ();
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.keyDown())
  {
    switch (event.getKeyCode())
    {
      case ' ':    // space: grab a single frame
        record_single = true;
        break;
          
      case 'p':   // paused
        paused = !paused;
        break;
          
      case 'K':
        record_fixed_number = false;
        record_continuously = !record_continuously;
        if (record_continuously)
          std::cerr << "STARTED recording." << std::endl;
        else
          std::cerr << "STOPPED recording." << std::endl;
        break;

      case 'L':
        record_fixed_number = true;
        record_continuously = !record_continuously;
        if (record_continuously)
        {
          std::cerr << "STARTED recording." << std::endl;
          rec_nr_frames = 0;
        }
        else
          std::cerr << "STOPPED recording." << std::endl;
        break;
    }
  }
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "pcl_online_viewer");
  ros::NodeHandle nh;

  int queue_size = 1;
  pcl::console::parse_argument (argc, argv, "-qsize", queue_size);
  
  ros::Subscriber sub = nh.subscribe ("input", queue_size, cloud_cb);
  topic_name = ros::names::remap("input").c_str();
  pcl::console::print_highlight("Subscribing to %s using a queue size of %d\n", topic_name.c_str(), queue_size);
  
  pcl::visualization::PCLVisualizer p (argc, argv, "Online PointCloud2 Viewer");
  pcl::PointCloud<Point>::Ptr cloud_xyz(new pcl::PointCloud<Point>);
  ColorHandlerPtr color_handler;

  p.registerKeyboardCallback(keyboardEventOccurred, (void*)&p);

  int color_handler_idx = 0;
  double psize = 0;
  while (nh.ok ())
  {
    ros::spinOnce ();
    ros::Duration (0.001).sleep ();
    p.spinOnce (10);

    if (!cloud_)
      continue;

    if (cloud_ == cloud_old_)
      continue;

    color_handler_idx = p.getColorHandlerIndex ("cloud");
    p.getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud");
    p.removePointCloud ("cloud");
    m.lock ();
    {
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
      pcl::fromPCLPointCloud2 (*cloud_, *cloud_xyz);
#else
      pcl::fromROSMsg (*cloud_, *cloud_xyz);
#endif
      color_handler.reset (new pcl::visualization::PointCloudColorHandlerCustom<PointCloud2> (cloud_, 255.0, 1.0, 1.0));
      p.addPointCloud<Point>(cloud_xyz, color_handler, "cloud");
      for (size_t i = 0; i < cloud_->fields.size (); ++i)
      {
        if (cloud_->fields[i].name == "rgb" || cloud_->fields[i].name == "rgba")
          color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<PointCloud2> (cloud_));
        else
          color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<PointCloud2> (cloud_, cloud_->fields[i].name));
        p.addPointCloud<Point>(cloud_xyz, color_handler, "cloud");
      }
      p.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud");
      if (color_handler_idx != -1)
        p.updateColorHandlerIndex ("cloud", color_handler_idx);
      cloud_old_ = cloud_;
    }
    m.unlock ();
  }

  return (0);
}
