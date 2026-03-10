#ifndef COMMS_EMULATOR_HELPER_SYSTEM__COMMS_EMULATOR_HELPER_HH_
#define COMMS_EMULATOR_HELPER_SYSTEM__COMMS_EMULATOR_HELPER_HH_

// The only required include in the header for systems
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>

// For this plugin
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <memory>
#include <mutex>
#include <map>
#include <deque>
#include <algorithm>
#include <cmath>

namespace comms_emulator_helper_system
{
  // Position class to hold the x, y, z coordinates
  class Position 
  {
  public:
      // Constructor
      Position(double x = 0.0, double y = 0.0, double z = 0.0)
          : x_(x), y_(y), z_(z) {}

      // Setters
      void setPosition(double x, double y, double z) 
      {
          x_ = x;
          y_ = y;
          z_ = z;
      }

      // Getters
      double getX() const { return x_; }
      double getY() const { return y_; }
      double getZ() const { return z_; }

      // Euclidean distance between two Position objects
      double distanceTo(const Position& other) const 
      {
          return std::sqrt(
              std::pow(other.getX() - this->x_, 2) + 
              std::pow(other.getY() - this->y_, 2) + 
              std::pow(other.getZ() - this->z_, 2)
          );
      }

      // Vector subtraction
      Position operator-(const Position &other) const 
      {
          return Position(this->x_ - other.x_, this->y_ - other.y_, this->z_ - other.z_);
      }

      // Dot product
      double dot(const Position &other) const 
      {
          return this->x_ * other.x_ + this->y_ * other.y_ + this->z_ * other.z_;
      }

      // Squared length of the vector
      double squaredLength() const 
      {
          return x_ * x_ + y_ * y_ + z_ * z_;
      }

      // 2D Vector subtraction
      Position subtract2D(const Position &other) const 
      {
          return Position(this->x_ - other.x_, this->y_ - other.y_, 0.0);
      }

      // 2D Dot product
      double dot2D(const Position &other) const 
      {
          return this->x_ * other.x_ + this->y_ * other.y_;
      }

      // Squared length of the vector in 2D
      double squaredLength2D() const 
      {
          return x_ * x_ + y_ * y_;
      }

  private:
      double x_;
      double y_;
      double z_;
  };

  // RobotNetworkConfig class to manage robot specific communication parameters
  class RobotNetworkConfig 
  {
  public:
      // Antenna noise floor is taken from https://www.montana.edu/aolson/ee447/EB%20and%20NO.pdf for 802.11n wifi 
      // This should come from antenna specs; for now, let's keep this.
      RobotNetworkConfig(double txPower = 30.0, double antennaNoiseFloor = -101.0) 
          : txPower_(txPower), antennaNoiseFloor_(antennaNoiseFloor) {}

      double getTxPower() const { return txPower_; }
      double getAntennaNoiseFloor() const { return antennaNoiseFloor_; }

      void setTxPower(double txPower) { txPower_ = txPower; }
      void setAntennaNoiseFloor(double antennaNoiseFloor) { antennaNoiseFloor_ = antennaNoiseFloor;}

  private:
      double txPower_;
      double antennaNoiseFloor_;
  };

  // EnvNetworkConfig class to manage path loss communication parameters
  class EnvNetworkConfig 
  {
  public:
      EnvNetworkConfig(double l0 = -30.0, double fadingExponent = 3.5, double variance = 10.0) 
          : l0_(l0), fadingExponent_(fadingExponent), variance_(variance) {}

      double getL0() const { return l0_; }
      double getFadingExponent() const { return fadingExponent_; }
      double getVariance() const { return variance_; }

      void setL0(double l0) { l0_ = l0; }
      void setFadingExponent(double fadingExponent) { fadingExponent_ = fadingExponent; }
      void setVariance(double variance) { variance_ = variance; }

  private:
      double l0_;
      double fadingExponent_;
      double variance_;
  };

  /// \brief Private data class for CommsEmulatorHelper
  class CommsEmulatorHelperPrivate
  {
    public:
      gz::transport::Node node;  // Transport node for communication
      ignition::msgs::Pose_V receivedData;  // Data from subscription
      std::mutex dataMutex;  // Mutex to protect access to the received data
      std::map<std::string, Position> robotPositions; // Map to store positions by robot name
      std::map<std::string, Position> treePositions; // Map to store tree positions
      std::map<std::string, std::map<std::string, gz::transport::Node::Publisher>> packetErrorRatePublishers;
      std::map<std::string, std::map<std::string, gz::transport::Node::Publisher>> packetDropRatePublishers;
      std::map<std::string, std::map<std::string, gz::transport::Node::Publisher>> pathLossPublishers;
      std::map<std::string, std::map<std::string, gz::transport::Node::Publisher>> RSSPublishers;
      std::map<std::string, std::map<std::string, gz::transport::Node::Publisher>> DelayPublishers;
      std::map<std::string, std::map<std::string, gz::transport::Node::Publisher>> BandwidthPublishers;
      
      // New members for SNR history and previous bandwidth state per robot pair
      std::map<std::string, std::map<std::string, std::deque<double>>> snrHistory;
      std::map<std::string, std::map<std::string, double>> prevBandwidth;
  };

  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
  // plugins that want to send commands.
  class CommsEmulatorHelper:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate
  {
    public: CommsEmulatorHelper();

    // Plugins inheriting ISystemConfigure must implement the Configure callback.
    public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager) override;

    // Plugins inheriting ISystemPreUpdate must implement the PreUpdate callback.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    // Plugins inheriting ISystemUpdate must implement the Update callback.
    public: void Update(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate callback.
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    // Callback function for topic subscription
    public: void OnPoseInfoTopic(const ignition::msgs::Pose_V &_msg);

    // Declare dataPtr in the class
    private:
      std::unique_ptr<CommsEmulatorHelperPrivate> dataPtr;  
  };
}

#endif
