/*
 * RosVisualization.hpp
 *
 *  Created on: Jul 4, 2017
 *      Author: Péter Fankhauser
 */

#pragma once

#include <free_gait_core/free_gait_core.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

namespace free_gait {

class RosVisualization
{
 public:
  RosVisualization();
  virtual ~RosVisualization();

  static const visualization_msgs::Marker getStanceMarker(const Stance& stance, const std::string& frameId,
                                                          const std_msgs::ColorRGBA& color);

  static const visualization_msgs::Marker getComMarker(const Position& comPosition, const std::string& frameId,
                                                       const std_msgs::ColorRGBA& color, const double size);

  static const visualization_msgs::MarkerArray getComWithProjectionMarker(const Position& comPosition,
                                                                          const std::string& frameId,
                                                                          const std_msgs::ColorRGBA& color,
                                                                          const double comMarkerSize,
                                                                          const double projectionLenght,
                                                                          const double projectionDiameter);
  static const visualization_msgs::MarkerArray getFootholdsMarker(const Stance& footholds, const std::string& frameId,
                                                                  const std_msgs::ColorRGBA& color, const double size);
  static const std::string getFootName(const LimbEnum& limb);
};

}
