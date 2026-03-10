// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>
#include <deque>
#include <map>
#include <algorithm>
#include <random>

//kalhan
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <random>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// plugin's header.
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
  // TODO: get world name from SDF file.
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
      std::string topicRSS = "/robot_comms_emu_helper/" + robot1 + "_to_" + robot2 + "/rx_strength";
      std::string topicDelay = "/robot_comms_emu_helper/" + robot1 + "_to_" + robot2 + "/delay";
      std::string topicBandwidth = "/robot_comms_emu_helper/" + robot1 + "_to_" + robot2 + "/bandwidth";

      // Create publishers and store them
      this->dataPtr->packetErrorRatePublishers[robot1][robot2] = this->dataPtr->node.Advertise<gz::msgs::Double>(topicPER, options);
      this->dataPtr->packetDropRatePublishers[robot1][robot2] = this->dataPtr->node.Advertise<gz::msgs::Double>(topicPDR, options);
      this->dataPtr->pathLossPublishers[robot1][robot2] = this->dataPtr->node.Advertise<gz::msgs::Double>(topicPathLoss, options);
      this->dataPtr->RSSPublishers[robot1][robot2] = this->dataPtr->node.Advertise<gz::msgs::Double>(topicRSS, options);
      this->dataPtr->DelayPublishers[robot1][robot2] = this->dataPtr->node.Advertise<gz::msgs::Double>(topicDelay, options);
      this->dataPtr->BandwidthPublishers[robot1][robot2] = this->dataPtr->node.Advertise<gz::msgs::Double>(topicBandwidth, options);
    }
  }
}

// Subscription callback for the pose/info topic
void CommsEmulatorHelper::OnPoseInfoTopic(const gz::msgs::Pose_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);
  this->dataPtr->receivedData = _msg;
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
  else if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "Tree positions updated only once in the beginning!." << std::endl;
  }
}

void CommsEmulatorHelper::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm)
{
  // For future use maybe.
}


// Calculate Path Loss, according to https://ieeexplore.ieee.org/document/9260568
double CalculateTreePathLoss(double distance, double numTrees) 
{
  double Po = 49.17; // RF power (Output Power)=17 dBm, Antenna Gain=1.5 dBi, Tx Power=17+1.5=18.5 dBm, RSSI 1m = -30.67 dBm
  double Lv = 11.98; // Tree attenuation in dB
  double sigma = 4.8;    // Standard deviation (dB) based on provided values in the paper

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist(0.0, sigma);
  double randomOffset = dist(gen);

  return Po + 20 * std::log10(distance) + numTrees * Lv + randomOffset;
}

// Convert dBm to Power
double DbmToPow(double dBm) {
  return 0.001 * pow(10., dBm / 10.);
}

// Calculate SNR to BER for QPSK
double QPSKSNRToBER(double power, double noise) {
  return erfc(sqrt(power / noise));
}

// Calculate SNR to BER for 64-QAM (Rayleigh)
double RAYLEIGHQAM64SNRToBER(double power, double noise) {
    double data_rate = 72e6;   // 72 Mbps
    double bandwidth = 20e6;   // 20 MHz
    double spectral_efficiency = data_rate / bandwidth;
    double EbNo = (power / noise) / spectral_efficiency;

    double term1 = (7.0 / 24.0) * (1 - std::sqrt((1.0 / 7.0) * EbNo / (1.0 + (1.0 / 7.0) * EbNo)));
    double term2 = (1.0 / 4.0) * (1 - std::sqrt((9.0 / 7.0) * EbNo / (1.0 + (9.0 / 7.0) * EbNo)));
    double term3 = -(1.0 / 24.0) * (1 - std::sqrt((25.0 / 7.0) * EbNo / (1.0 + (25.0 / 7.0) * EbNo)));
    double term4 = (1.0 / 24.0) * (1 - std::sqrt((81.0 / 7.0) * EbNo / (1.0 + (81.0 / 7.0) * EbNo)));
    double term5 = -(1.0 / 24.0) * (1 - std::sqrt((169.0 / 7.0) * EbNo / (1.0 + (169.0 / 7.0) * EbNo)));

    return term1 + term2 + term3 + term4 + term5;
}

// Calculate SNR to BER for 64-QAM (AWGN)
double AWGNQAM64EbNoToBER(double power, double noise) {
    double M = 64.0;
    int k = 6; // For 64-QAM, k = log2(64) = 6
    double data_rate = 72e6;   // 72 Mbps
    double bandwidth = 20e6;   // 20 MHz
    double spectral_efficiency = data_rate / bandwidth;
    double EbNo = (power / noise) / spectral_efficiency;
    double factor = (4.0 / k) * (1.0 - 1.0 / std::sqrt(M)) * 0.5;
    double x = std::sqrt((3.0 * k * EbNo) / (M - 1.0));
    double ber = factor * std::erfc(x / std::sqrt(2.0));
    return ber;
}

// Calculate BER to PER
double BERToPER(double ber, int packetSizeBits) {
  return 1 - pow(1 - ber, packetSizeBits);
}

double PointToSegmentDistanceSquared(const Position &A, const Position &B, const Position &C)
{
  Position AB = B.subtract2D(A);
  Position AC = C.subtract2D(A);
  Position BC = C.subtract2D(B);

  double e = AC.dot2D(AB);
  if (e <= 0.0)
      return AC.squaredLength2D(); // Closest to A
  double f = AB.squaredLength2D();
  if (e >= f)
      return BC.squaredLength2D(); // Closest to B
  return AC.squaredLength2D() - (e * e) / f;
}

double FresnelZoneRadius (double distanceMetres, double frequencyHz)
{
  return 0.5 * std::sqrt(3.0e8 / frequencyHz * distanceMetres);
}

void CommsEmulatorHelper::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) 
{
  std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);

  int poseCount = this->dataPtr->receivedData.pose_size();
  if (poseCount == 0) {
      igndbg << "No poses received to update in PostUpdate." << std::endl;
      return;
  }

  for (int i = 0; i < poseCount; ++i)
  {
    const auto &pose = this->dataPtr->receivedData.pose(i);
    auto robotPosIt = this->dataPtr->robotPositions.find(pose.name());
    if (robotPosIt != this->dataPtr->robotPositions.end())
    {
        robotPosIt->second.setPosition(
            pose.position().x(),
            pose.position().y(),
            pose.position().z()
        );
    }
  }

  if (this->dataPtr->robotPositions.size() < 2) {
      igndbg << "Not enough robots to calculate distances and network parameters." << std::endl;
      return;
  }

  EnvNetworkConfig envNetworkConfig;
  RobotNetworkConfig robotNetworkConfig;
  double packetSizeInBits = 524280.0;

  for (const auto &robot1Entry : this->dataPtr->robotPositions) 
  {
    const std::string &name1 = robot1Entry.first;
    const Position &pos1 = robot1Entry.second;

    for (const auto &robot2Entry : this->dataPtr->robotPositions) 
    {
      const std::string &name2 = robot2Entry.first;
      const Position &pos2 = robot2Entry.second;
      igndbg << "Calculating parameters between " << name1 << " and " << name2 << std::endl;

      if (name1 == name2) continue;
      
      double distance = pos1.distanceTo(pos2);
      igndbg << "Distance " << distance << std::endl;
      if (distance == 0) continue;

      double treeRadius = 0.3;
      double linkModelWidth = FresnelZoneRadius(distance, 2.4e9);
      double onTheLinkTreshold = treeRadius + linkModelWidth;
      
      int treesOnLink = 0;
      for (const auto &treeEntry : this->dataPtr->treePositions) 
      {
          const Position &treePos = treeEntry.second;
          double distanceSquared = PointToSegmentDistanceSquared(pos1, pos2, treePos);
          if (distanceSquared < onTheLinkTreshold * onTheLinkTreshold) 
          {
              ++treesOnLink;
          }
      }
      igndbg << "Trees on link " << treesOnLink << std::endl;

      double pathLoss = CalculateTreePathLoss(distance, treesOnLink);
      igndbg << "Path loss " << pathLoss << std::endl;

      double rxPower = robotNetworkConfig.getTxPower() - pathLoss;
      double ber = 0.0;
      if (treesOnLink == 0)
      {
          ber = AWGNQAM64EbNoToBER(DbmToPow(rxPower), DbmToPow(robotNetworkConfig.getAntennaNoiseFloor()));
      }
      else
      {
          ber = RAYLEIGHQAM64SNRToBER(DbmToPow(rxPower), DbmToPow(robotNetworkConfig.getAntennaNoiseFloor()));
      }
      double per = BERToPER(ber, packetSizeInBits);

      double SNR = rxPower - robotNetworkConfig.getAntennaNoiseFloor();

      if (SNR < 2) 
      {
        gz::msgs::Double msgPDR;
        msgPDR.set_data(1.0);
        if (this->dataPtr->packetDropRatePublishers.count(name1) > 0 &&
            this->dataPtr->packetDropRatePublishers[name1].count(name2) > 0) 
        {
            this->dataPtr->packetDropRatePublishers[name1][name2].Publish(msgPDR);
            igndbg << "PDR " << name1 << " and " << name2 << ": 1.0" << std::endl;
        }
      }
      else
      {
        gz::msgs::Double msgPDR;
        msgPDR.set_data(1e-08);
        if (this->dataPtr->packetDropRatePublishers.count(name1) > 0 &&
            this->dataPtr->packetDropRatePublishers[name1].count(name2) > 0) 
        {
            this->dataPtr->packetDropRatePublishers[name1][name2].Publish(msgPDR);
            igndbg << "PDR " << name1 << " and " << name2 << ": 0.0" << std::endl;
        }
      }

      gz::msgs::Double msgPER;
      msgPER.set_data(per);
      gz::msgs::Double msgPathLoss;
      msgPathLoss.set_data(pathLoss);

      if (this->dataPtr->packetErrorRatePublishers.count(name1) > 0 &&
          this->dataPtr->packetErrorRatePublishers[name1].count(name2) > 0) 
      {
          this->dataPtr->packetErrorRatePublishers[name1][name2].Publish(msgPER);
      }

      if (this->dataPtr->pathLossPublishers.count(name1) > 0 &&
          this->dataPtr->pathLossPublishers[name1].count(name2) > 0) 
      {
          this->dataPtr->pathLossPublishers[name1][name2].Publish(msgPathLoss);
      }

      gz::msgs::Double msgRSS;
      msgRSS.set_data(rxPower);
      if (this->dataPtr->RSSPublishers.count(name1) > 0 &&
          this->dataPtr->RSSPublishers[name1].count(name2) > 0) 
      {
          this->dataPtr->RSSPublishers[name1][name2].Publish(msgRSS);
      }

      gz::msgs::Double msgDelay;
      double Delay = 2.5; // ms
      msgDelay.set_data(Delay);
      if (this->dataPtr->DelayPublishers.count(name1) > 0 &&
          this->dataPtr->DelayPublishers[name1].count(name2) > 0) 
      {
          this->dataPtr->DelayPublishers[name1][name2].Publish(msgDelay);
      }

      // --- Updated Bandwidth Calculation Using SNR History and State Transitions ---

    // Record the current SNR for this robot pair.
    auto &history = this->dataPtr->snrHistory[name1][name2];
    history.push_back(SNR);
    if (history.size() > 8)
    {
        history.pop_front();
    }

    // Retrieve the current (previous) bandwidth state.
    // Default to 72 if not set.
    double currentBandwidth = this->dataPtr->prevBandwidth[name1][name2];
    if (currentBandwidth != 72 && currentBandwidth != 28.9 && currentBandwidth != 7.2 && currentBandwidth != 0)
    {
        currentBandwidth = 72;
    }

    double newBandwidth = currentBandwidth;

    // Use the history decision only if we have at least 8 samples.
    if (history.size() >= 8)
    {
        int countHigh = 0;
        int countLow = 0;
        double highThreshold, lowThreshold;

        // Set thresholds based on the current bandwidth.
        if (currentBandwidth == 72)
        {
            highThreshold = 25;
            lowThreshold = 25;
        }
        else if (currentBandwidth == 28.9)
        {
            highThreshold = 11;
            lowThreshold = 11;
        }
        else if (currentBandwidth == 7.2)
        {
            highThreshold = 2;
            lowThreshold = 2;
        }
        else if (currentBandwidth == 0)
        {
            // For BW of 0, use the same thresholds as for 7.2.
            highThreshold = 2;
            lowThreshold = 2;
        }
        
        // Count high and low SNR samples based on the selected thresholds.
        for (const auto &val : history)
        {
            if (val > highThreshold)
                countHigh++;
            if (val < lowThreshold)
                countLow++;
        }
        
        if (currentBandwidth == 72)
        {
            // From 72, drop to 28.9 if at least 3 low SNR samples.
            if (countLow >= 3)
                newBandwidth = 28.9;
            else
                newBandwidth = 72;
        }
        else if (currentBandwidth == 28.9)
        {
            // From 28.9, switch upward if all 8 samples are high;
            // drop to 7.2 if at least 3 samples are low.
            if (countHigh == 8)
                newBandwidth = 72;
            else if (countLow >= 3)
                newBandwidth = 7.2;
            else
                newBandwidth = 28.9;
        }
        else if (currentBandwidth == 7.2)
        {
            // From 7.2, switch upward if all 8 are high;
            // drop to 0 if at least 3 are low.
            if (countHigh == 8)
                newBandwidth = 28.9;
            else if (countLow >= 3)
                newBandwidth = 0;
            else
                newBandwidth = 7.2;
        }
        else if (currentBandwidth == 0)
        {
            // From 0, switch upward to 7.2 if all 8 samples are high;
            // otherwise, remain at 0.
            if (countHigh == 8)
                newBandwidth = 7.2;
            else
                newBandwidth = 0;
        }
    }
    else
    {
        // Fewer than 8 samples: use an instantaneous decision.
        if (SNR > 25)
            newBandwidth = 72;
        else if (SNR > 11)
            newBandwidth = 28.9;
        else if (SNR > 2)
            newBandwidth = 7.2;
        else
            newBandwidth = 0;
    }

    // Save the new bandwidth for future iterations.
    this->dataPtr->prevBandwidth[name1][name2] = newBandwidth;

    gz::msgs::Double msgBandwidth;
    msgBandwidth.set_data(newBandwidth);
    if (this->dataPtr->BandwidthPublishers.count(name1) > 0 &&
        this->dataPtr->BandwidthPublishers[name1].count(name2) > 0) 
    {
        this->dataPtr->BandwidthPublishers[name1][name2].Publish(msgBandwidth);
    }


    }
  }
}

}  // namespace comms_emulator_helper_system
