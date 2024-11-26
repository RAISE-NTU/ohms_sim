
// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>

//kalhan
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <random>


// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "comms_emulator_helper_system/CommsEmulatorHelper.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    comms_emulator_helper_system::CommsEmulatorHelper,
    gz::sim::System,
    comms_emulator_helper_system::CommsEmulatorHelper::ISystemConfigure,
    comms_emulator_helper_system::CommsEmulatorHelper::ISystemPreUpdate,
    comms_emulator_helper_system::CommsEmulatorHelper::ISystemUpdate,
    comms_emulator_helper_system::CommsEmulatorHelper::ISystemPostUpdate
)

namespace comms_emulator_helper_system 
{

CommsEmulatorHelper::CommsEmulatorHelper()
      : dataPtr(std::make_unique<CommsEmulatorHelperPrivate>())
{
}

void CommsEmulatorHelper::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager)
{
  // Subscribe to the ../pose/info topic
  this->dataPtr->node.Subscribe("/world/marsyard2020/pose/info", 
                                &CommsEmulatorHelper::OnPoseInfoTopic, this);

  // Hard-code robot names for now. 
  // TODO: read these from SDF file.
  std::vector<std::string> robotNames = {"atlas", "bestla"};

  // Initialize positions for each robot name
  for (const auto &robotName : robotNames) 
  {
      this->dataPtr->robotPositions[robotName] = Position();
  }

  // Set the desired message rate (messages per second)
  // Otherwise emulator will lag behind with a huge queue (5 works for now)
  uint64_t desiredMsgsPerSec = 5;  

  // Configure AdvertiseMessageOptions
  gz::transport::AdvertiseMessageOptions options;
  options.SetMsgsPerSec(desiredMsgsPerSec);

  // Set up publishers for each possible pair of robots (excluding self-pairs)
  for (const auto &robot1 : robotNames) {
    for (const auto &robot2 : robotNames) 
    {
      if (robot1 == robot2) continue; // Skip self-pairs

      // Define topic names
      std::string topicPER = "/robot_comms_emu_helper/" + robot1 + "_to_" + robot2 + "/packet_error_rate";
      std::string topicPDR = "/robot_comms_emu_helper/" + robot1 + "_to_" + robot2 + "/packet_drop_rate";
      std::string topicPathLoss = "/robot_comms_emu_helper/" + robot1 + "_to_" + robot2 + "/path_loss";

      // Create publishers and store them
      this->dataPtr->packetErrorRatePublishers[robot1][robot2] = this->dataPtr->node.Advertise<gz::msgs::Double>(topicPER, options);
      this->dataPtr->packetDropRatePublishers[robot1][robot2] = this->dataPtr->node.Advertise<gz::msgs::Double>(topicPDR, options);
      this->dataPtr->pathLossPublishers[robot1][robot2] = this->dataPtr->node.Advertise<gz::msgs::Double>(topicPathLoss, options);
      
      //igndbg << "Created publishers for robot pair: " << robot1 << " to " << robot2 << std::endl;
    }
  }
}

// Subscription callback for the example topic
void CommsEmulatorHelper::OnPoseInfoTopic(const gz::msgs::Pose_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);
  this->dataPtr->receivedData = _msg;

  //Debug: Print the number of poses received
  //igndbg << "Received " << _msg.pose_size() << " poses in OnPoseInfoTopic" << std::endl;
  // for (int i = 0; i < _msg.pose_size(); ++i) 
  // {
  //     const auto &pose = _msg.pose(i);
  //     if (pose.name().find("tree") != std::string::npos)
  //     {
  //       igndbg << "Pose received for: " << pose.name() 
  //             << " Position: (" << pose.position().x() 
  //             << ", " << pose.position().y() 
  //             << ", " << pose.position().z() << ")" << std::endl;
  //     }
  // }
}

void CommsEmulatorHelper::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
  // Trees are static objects! So updating once. 
  // TODO: Add a configurable condition here to update for dynamic objects. However it is unstable! (Out of bound error) 
  // Get positions of static models into a data structure with model name and position
  if (this->dataPtr->treePositions.empty())
  {
    for (int i = 0; i < this->dataPtr->receivedData.pose_size(); ++i)
    {
      const auto &pose = this->dataPtr->receivedData.pose(i);

      // Check if the name contains "tree" to identify tree models
      if (pose.name().find("tree") != std::string::npos)
      {
        // Extract position and save it to treePositions map
        Position treePosition(pose.position().x(), pose.position().y(), pose.position().z());
        this->dataPtr->treePositions[pose.name()] = treePosition;      
      }
    }
  }
  else
    if (!_info.paused && _info.iterations % 1000 == 0)
    {
      igndbg << "Tree positions updated only once in the beginning!." << std::endl;
    }
}

void CommsEmulatorHelper::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm)
{
  // For future use maybe.
}

// Function to calculate path loss with variance
double CalculatePathLossWithVariance(double distance, const EnvNetworkConfig &networkConfig) 
{

    double pathLoss = networkConfig.getL0() + 
                      10 * networkConfig.getFadingExponent() * std::log10(distance);

    // Random number generation for variance
    std::random_device rd;         
    std::mt19937 gen(rd());          
    std::normal_distribution<> noise(0.0, networkConfig.getVariance());

    // Add random noise to simulate variance
    pathLoss += noise(gen);

    return pathLoss;
}

// Calculate Free Space Path Loss
double CalculateFreeSpacePathLoss(double distance) 
{
    return 20 * std::log10(distance) + 20 * std::log10(2.4) - 27.55;
}

// Convert dBm to Power
double DbmToPow(double dBm) {
  return 0.001 * pow(10.,dBm/10.);
}

// Calculate (QPSK) SNR to BER
double QPSKSNRToBER(double power, double noise) {
  return erfc(sqrt(power / noise));
}

// Calculate BER to PER
double BERToPER(double ber, int packetSizeBits) {
  return 1 - pow(1 - ber, packetSizeBits);
}

double PointToSegmentDistanceSquared(const Position &A, const Position &B, const Position &C)
{
  // Calculations are made in 2D projections.
  // Position class holds methods to calculate in 3D as well.
  // Some objects will need 3D, tree trunks are okay with 2D.
  // Maybe for walls 1D with other conditions!
  Position AB = B.subtract2D(A);
  Position AC = C.subtract2D(A);
  Position BC = C.subtract2D(B);

  double e = AC.dot2D(AB);
  // Handle cases where projection falls outside the segment
  if (e <= 0.0)
      return AC.squaredLength2D(); // Closest to A
  double f = AB.squaredLength2D();
  if (e >= f)
      return BC.squaredLength2D(); // Closest to B
  // Projection falls within the segment
  return AC.squaredLength2D() - (e * e) / f;
}

double FresnelZoneRadius (double distanceMetres, double frequencyHz)
{
  // Calculate the first Fresnel zone radius (F = 0.5 * sqrt(wavelength * distance)), wavelength (λ = c / f)
  return 0.5 * std::sqrt(3.0e8 / frequencyHz * distanceMetres);
}

void CommsEmulatorHelper::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) 
{
  // if (!_info.paused && _info.iterations % 1000 == 0)
  // {
  //   igndbg << "comms_emulator_helper_system::CommsEmulatorHelper::PostUpdate" << std::endl;
  // }

  // Lock for thread safety when accessing shared data
  std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);

  // Ensure that there are poses to process
  int poseCount = this->dataPtr->receivedData.pose_size();
  if (poseCount == 0) {
      igndbg << "No poses received to update in PostUpdate." << std::endl;
      return;
  }

  // Loop through each pose in receivedData, with index bounds check
  for (int i = 0; i < poseCount; ++i)
  {
    if (i >= poseCount) 
    {
        ignerr << "Out-of-bounds access attempted in PostUpdate: index " << i << ", total poses " << poseCount << std::endl;
        break;
    }

    const auto &pose = this->dataPtr->receivedData.pose(i);

    // Verify if pose name matches any key in robotPositions
    auto robotPosIt = this->dataPtr->robotPositions.find(pose.name());
    if (robotPosIt != this->dataPtr->robotPositions.end())
    {
        // Update the robot's position
        robotPosIt->second.setPosition(
            pose.position().x(),
            pose.position().y(),
            pose.position().z()
        );
    }
  }

  // Debugging: Print the entire robotPositions map after processing updates
  // igndbg << "=== Current State of robotPositions Map ===" << std::endl;
  // for (const auto &entry : this->dataPtr->robotPositions) {
  //     const auto &name = entry.first;
  //     const auto &pos = entry.second;
  //     igndbg << "Robot: " << name 
  //             << " | Position: (" << pos.getX() 
  //             << ", " << pos.getY() 
  //             << ", " << pos.getZ() << ")" << std::endl;
  // }
  // igndbg << "===========================================" << std::endl;

  // Debugging: Print the entire treePositions map 
  // igndbg << "=== Current State of treePositions Map ===" << std::endl;
  // for (const auto &entry : this->dataPtr->treePositions) {
  //     const auto &name = entry.first;
  //     const auto &pos = entry.second;
  //     igndbg << "Tree: " << name 
  //             << " | Position: (" << pos.getX() 
  //             << ", " << pos.getY() 
  //             << ", " << pos.getZ() << ")" << std::endl;
  // }
  // igndbg << "===========================================" << std::endl;

  // Ensure there are robot positions to process
  if (this->dataPtr->robotPositions.size() < 2) {
      igndbg << "Not enough robots to calculate distances and network parameters." << std::endl;
      return;
  }

  // Instantiate the EnvNetworkConfig (adjust parameters as needed)
  EnvNetworkConfig envNetworkConfig;
  RobotNetworkConfig robotNetworkConfig;

  // Packet size in bits for PER calculation
  double packetSizeInBits = 12000;

  // Loop over all pairs of robots (excluding self-pairs)
  for (const auto &robot1Entry : this->dataPtr->robotPositions) 
  {
    const std::string &name1 = robot1Entry.first;
    const Position &pos1 = robot1Entry.second;

    for (const auto &robot2Entry : this->dataPtr->robotPositions) 
    {
      const std::string &name2 = robot2Entry.first;
      const Position &pos2 = robot2Entry.second;

      if (name1 == name2) continue; // Skip self-pairs

      // Calculate Euclidean distance between the two robots
      double distance = pos1.distanceTo(pos2);
      
      if (distance == 0) continue; // To avoid edge case calculations. dividing by zero

      // **Calculate number of trees in LOS (LINK model: Estimating Low-Power Radio Signal Attenuation in Forests: A LiDAR-Based Approach )**
      double treeRadius = 0.3; // Adjust based on actual tree size ( here circumference is taken as 1m )
      double linkModelWidth = FresnelZoneRadius(distance, 2.4e9); // Width of the LINK model = Radius of first Fresnel Zone
      double onTheLinkTreshold = treeRadius + linkModelWidth; // Threshold distance to consider a tree "on the line"
      
      int treesOnLink = 0;

      // Iterate over all trees
      for (const auto &treeEntry : this->dataPtr->treePositions) 
      {
          const Position &treePos = treeEntry.second;

          // Debug the robots and tree positions
          // igndbg << "Checking for (" << pos1.getX() << ", " << pos1.getY() << ") and ("
          // << pos2.getX() << ", " << pos2.getY() << ") with ("
          // << treePos.getX() << ", " << treePos.getY() << ")" << std::endl;

          // Calculate squared distance from the tree to the line segment between the two robots
          double distanceSquared = PointToSegmentDistanceSquared(pos1, pos2, treePos);

          //Debug distance
          // igndbg << "Squared distance from the tree to the line segment: " << distanceSquared <<
          // " Squared on the link threshold : " << onTheLinkTreshold * onTheLinkTreshold << std::endl;

          // If the distance is less than or equal to the squared tree radius, count the tree
          if (distanceSquared < onTheLinkTreshold * onTheLinkTreshold) 
          {
              ++treesOnLink;
          }
      }

      // Output the number of trees on the line
      //igndbg << "Number of trees on the line between " << name1 << " and " << name2 << ": " << treesOnLink << std::endl;

      double pathLoss = 0;
      
      // Calculate NetworkConfig parameters
      if (treesOnLink == 0)
      {
          pathLoss = CalculateFreeSpacePathLoss(distance);
      }

      if (treesOnLink > 0)
      {
          double vegetationIndex = treesOnLink / (linkModelWidth * distance) * treeRadius * 2; // VD = TD.D
          envNetworkConfig.setL0(-0.82 * vegetationIndex + 40.1);
          envNetworkConfig.setFadingExponent(0.1717 * vegetationIndex + 2.2043);
          envNetworkConfig.setVariance(4.4);
          pathLoss = CalculatePathLossWithVariance(distance, envNetworkConfig);
      }

      // Calculate received power
      double rxPower = robotNetworkConfig.getTxPower() - pathLoss;
      //igndbg << "Rx Power " << name1 << " and " << name2 << " : " << rxPower << std::endl;

      // // Message dropping because of antenna sensitivity
      // if (rxPower < robotNetworkConfig.getAntennaSensitivity())
      // {
      //   gz::msgs::Double msgPDR;
      //   msgPDR.set_data(1.0);

      //   // Publish to corresponding topics if publishers exist
      //   if (this->dataPtr->packetDropRatePublishers.count(name1) > 0 &&
      //       this->dataPtr->packetDropRatePublishers[name1].count(name2) > 0) 
      //   {
      //       this->dataPtr->packetDropRatePublishers[name1][name2].Publish(msgPDR);
      //       //igndbg << "Publish PDR " << name1 << " and " << name2 << ": 1.0" << std::endl;
      //   }
      // }
      // else
      // {
      //   gz::msgs::Double msgPDR;
      //   msgPDR.set_data(1e-08); // Don't change! This is the lowest we can go 
      //   // (I think because of gz_msg Double or std to_string limitations in the emulator)

      //   // Publish to corresponding topics if publishers exist
      //   if (this->dataPtr->packetDropRatePublishers.count(name1) > 0 &&
      //       this->dataPtr->packetDropRatePublishers[name1].count(name2) > 0) 
      //   {
      //       this->dataPtr->packetDropRatePublishers[name1][name2].Publish(msgPDR);
      //       //igndbg << "Publish PDR " << name1 << " and " << name2 << ": 1.0" << std::endl;
      //   }
      // }

      // Calculate BER using SNR
      double ber = QPSKSNRToBER(DbmToPow(rxPower), DbmToPow(envNetworkConfig.getSnrThreshold()));

      // Calculate PER using BER
      double per = BERToPER(ber, packetSizeInBits);

      // Consolidated debug message for all calculated values
      // igndbg << "Calculations between " << name1 << " and " << name2 << ":\n"
      //         << "  Euclidean Distance: " << distance << "\n"
      //         << "  Path Loss: " << pathLoss << "\n"
      //         << "  Rx Power: " << rxPower << "\n"
      //         << "  BER: " << ber << "\n"
      //         << "  PER: " << per << std::endl;

      // Not needed now!
      // Store network config for each robot pair
      // EmuNetworkConfig emuNetworkConfig;
      // emuNetworkConfig.setDistance(distance);
      // emuNetworkConfig.setPathLoss(pathLoss);
      // emuNetworkConfig.setRxPower(rxPower);
      // emuNetworkConfig.setBER(ber);
      // emuNetworkConfig.setPER(per);
      // this->dataPtr->robotPairNetworkConfigs[name1][name2] = emuNetworkConfig;
      
      
      
      if (per > 0.08) //robotNetworkConfig.getAntennaSensitivityPER() //from IEEE 802.11 standard PER should be 8% or 10% to achieve comms
      {
        gz::msgs::Double msgPDR;
        msgPDR.set_data(1.0);

        // Publish to corresponding topics if publishers exist
        if (this->dataPtr->packetDropRatePublishers.count(name1) > 0 &&
            this->dataPtr->packetDropRatePublishers[name1].count(name2) > 0) 
        {
            this->dataPtr->packetDropRatePublishers[name1][name2].Publish(msgPDR);
            igndbg << "PDR " << name1 << " and " << name2 << ": 1.0" << std::endl;
            igndbg << "Comms Status:" << name1 << " and " << name2 << " : " << "COMMS NOT AVAILABLE!" << std::endl;
        }
      }
      else
      {
        gz::msgs::Double msgPDR;
        msgPDR.set_data(1e-08); // Don't change! This is the lowest we can go 
        // (I think because of gz_msg Double or std to_string limitations in the emulator)

        // Publish to corresponding topics if publishers exist
        if (this->dataPtr->packetDropRatePublishers.count(name1) > 0 &&
            this->dataPtr->packetDropRatePublishers[name1].count(name2) > 0) 
        {
            this->dataPtr->packetDropRatePublishers[name1][name2].Publish(msgPDR);
            igndbg << "PDR " << name1 << " and " << name2 << ": 1.0" << std::endl;
            igndbg << "Comms Status:" << name1 << " and " << name2 << " : " << "COMMS AVAILABLE!" << std::endl;
        }
      }

      // Publish the calculated PER and Path Loss to the corresponding topics
      gz::msgs::Double msgPER;
      msgPER.set_data(per);
      gz::msgs::Double msgPathLoss;
      msgPathLoss.set_data(pathLoss);

      // Publish to corresponding topics if publishers exist
      if (this->dataPtr->packetErrorRatePublishers.count(name1) > 0 &&
          this->dataPtr->packetErrorRatePublishers[name1].count(name2) > 0) 
      {
          this->dataPtr->packetErrorRatePublishers[name1][name2].Publish(msgPER);
          igndbg << "PER:" << name1 << " and " << name2 << " : " << per << std::endl;
      }

      if (this->dataPtr->pathLossPublishers.count(name1) > 0 &&
          this->dataPtr->pathLossPublishers[name1].count(name2) > 0) 
      {
          this->dataPtr->pathLossPublishers[name1][name2].Publish(msgPathLoss);
          igndbg << "Path Loss:" << name1 << " and " << name2 << " : " << pathLoss << std::endl;
      }
      
    }
  }

}

}  // namespace comms_emulator_helper_system