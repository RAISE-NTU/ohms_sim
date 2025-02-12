
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
  std::vector<std::string> robotNames = {"atlas", "bestla", "rama", "ravana"};

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

// Calculate Meadow Path Loss
// https://ieeexplore.ieee.org/document/6484898
// Table IX - 2.4 GHz, 0.9m height parameters
double CalculateMeadowPathLoss(double distance) 
{
    double dbreak = 22.0;

    if (distance < dbreak) {
        return 67.7 + 17.5 * std::log10(distance);
    } else {
        return 35.75 + 41.3 * std::log10(distance);
    }
}

// https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=d955fd45934529f7886a23e385891f3c8c905062
// https://www.mdpi.com/1424-8220/22/9/3267
double CalculateMAITURPathLoss(double distance)
{
    // Constants as given (from Salameh's results)
    const double A_M = 38.0;       // Maximum excess attenuation in dB
    const double R = 0.9;          // Initial slope of the attenuation curve in dB/m
    const double f_GHz = 2.4;      // Frequency in GHz
    const double f_MHz = f_GHz * 1000.0; // Convert GHz to MHz

    // Convert distance from metres to kilometres
    double d_km = distance / 1000.0;

    // Calculate the attenuation term
    double attenuationTerm = A_M * (1.0 - std::exp((-R * d_km) / A_M));

    // Combine terms according to the formula:
    // PL = A_M(1 - e^(-R d / A_M)) + 32.44 + 20 log10(d) + 20 log10(f)
    double pathLoss = attenuationTerm 
                      + 32.44 
                      + 20.0 * std::log10(d_km) 
                      + 20.0 * std::log10(f_MHz);

    return pathLoss;
}

// Calculate Free Space Path Loss, according to https://ieeexplore.ieee.org/document/9260568
double CalculateTreePathLoss(double distance, double numTrees) 
{
  double Po = 49.17; // RF power (Output Power)=17 dBm, Antenna Gain=1.5 dBi, Tx Power=17+1.5=18.5 dBm, RSSI 1m = -30.67 dBm
  double Lv = 11.98; // Tree attenuation in dB
  double sigma = 4.8;    // Standard deviation (dB) 

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist(0.0, sigma);
  double randomOffset = dist(gen);


  return Po + 20 * std::log10(distance) + numTrees * Lv + randomOffset;
}

// Convert dBm to Power
double DbmToPow(double dBm) {
  return 0.001 * pow(10.,dBm/10.);
}

// Calculate SNR to BER for QPSK
double QPSKSNRToBER(double power, double noise) {
  return erfc(sqrt(power / noise));
}


// Calculate SNR to BER for 64-QAM
// references:
// SNR from EbNo - https://wcours.gel.ulaval.ca/2020/a/GEL7014/default/5notes/index.chtml#:~:text=Sklar%20textbook%3A%203.1.4
// here we use - SNR = EbNo * spectral efficiency (in bits per second per hertz) 
// BER from EbNo - https://www.raymaps.com/index.php/theoretical-ber-of-m-qam-in-rayleigh-fading/
// According to above gamma_c/k = EbNo in raymaps equation.
double RAYLEIGHQAM64SNRToBER(double power, double noise) {
    //double M = 64.0;
    //int k = 6; // For 64-QAM, k = log2(64) = 6

    // Parameters for 802.11n scenario:
    // Single spatial stream, 64-QAM, highest coding rate (MCS7) = 5/6
    // Approximate data rate: 72 Mbps in a 20 MHz channel
    double data_rate = 72e6;   // 72 Mbps
    double bandwidth = 20e6;   // 20 MHz

    // Calculate spectral efficiency: bits/s/Hz
    double spectral_efficiency = data_rate / bandwidth;

    // Convert power/noise to Eb/No
    // Eb/No = (SNR) / spectral_efficiency = (power/noise) / spectral_efficiency
    double EbNo = (power / noise) / spectral_efficiency;

    // Simplified BER formula for 64-QAM
    double term1 = (7.0 / 24.0) * (1 - std::sqrt(1.0 / 7.0 * EbNo / (1.0 + 1.0 / 7.0 * EbNo)));
    double term2 = (1.0 / 4.0) * (1 - std::sqrt(9.0 / 7.0 * EbNo / (1.0 + 9.0 / 7.0 * EbNo)));
    double term3 = -(1.0 / 24.0) * (1 - std::sqrt(25.0 / 7.0 * EbNo / (1.0 + 25.0 / 7.0 * EbNo)));
    double term4 = (1.0 / 24.0) * (1 - std::sqrt(81.0 / 7.0 * EbNo / (1.0 + 81.0 / 7.0 * EbNo)));
    double term5 = -(1.0 / 24.0) * (1 - std::sqrt(169.0 / 7.0 * EbNo / (1.0 + 169.0 / 7.0 * EbNo)));

    return term1 + term2 + term3 + term4 + term5;
}

// references:
// https://www.raymaps.com/index.php/qam-theoretical-ber/
// assumption: WiFi 802.11n, single spatial stream, 64-QAM, 5/6 coding rate, Short guard interval.
double AWGNQAM64EbNoToBER(double power, double noise) {
    double M = 64.0;
    int k = 6; // For 64-QAM, k = log2(64) = 6

    // Parameters for 802.11n scenario:
    // Single spatial stream, 64-QAM, highest coding rate (MCS7) = 5/6
    // Approximate data rate: 65 Mbps in a 20 MHz channel
    double data_rate = 72e6;   // 72 Mbps
    double bandwidth = 20e6;   // 20 MHz

    // Calculate spectral efficiency: bits/s/Hz
    double spectral_efficiency = data_rate / bandwidth; // 65e6 / 20e6 = 3.25 bits/s/Hz

    // Convert power/noise to Eb/No
    // Eb/No = (SNR) / spectral_efficiency = (power/noise) / spectral_efficiency
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

  // Ensure there are robot positions to process
  if (this->dataPtr->robotPositions.size() < 2) {
      igndbg << "Not enough robots to calculate distances and network parameters." << std::endl;
      return;
  }

  // Instantiate the EnvNetworkConfig and RobotNetworkConfig (adjust parameters as needed)
  EnvNetworkConfig envNetworkConfig;
  RobotNetworkConfig robotNetworkConfig;

  // Packet size in bits for PER calculation
  // 802.11n wifi
  // Aggregate MAC Service Data Unit (A-MSDU): Maximum Payload Size: Up to 7,935 bytes = 63,480 bits.
  // Aggregate MAC Protocol Data Unit (A-MPDU): Maximum Payload Size: Up to 65,535 bytes = 524,280 bits. (better for forests)
  double packetSizeInBits = 524280.0;

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

          // Calculate squared distance from the tree to the line segment between the two robots
          double distanceSquared = PointToSegmentDistanceSquared(pos1, pos2, treePos);

          // If the distance is less than or equal to the squared tree radius, count the tree
          if (distanceSquared < onTheLinkTreshold * onTheLinkTreshold) 
          {
              ++treesOnLink;
          }
      }

      double pathLoss = 0;
      
      // Calculate Path Loss
      // Method 1: If there are trees on LOS, tree attenuation is added to meadow attenuation.
      // for meadows : https://ieeexplore.ieee.org/document/6484898
      // for trees seperately : https://ieeexplore.ieee.org/document/9260568
      ////////////////////////////////////////////////////////////////////////
      // Method 2: Demetri uses Azevedo's methods. I use LINK model by demetry and improve using Fresnel zones.
      // Demetri : https://ieeexplore.ieee.org/document/7165025 
      // Azevedo : https://ieeexplore.ieee.org/document/5751639 
      ////////////////////////////////////////////////////////////////////////
      // Method 3: Similar to Method 1 with Maximum Attenuation ITU-R model swapped for Meadow model
      // MA-ITU-R: https://www.mdpi.com/1424-8220/22/9/3267#:~:text=ITU%2DR%20Maximum%20Attenuation%20and%20Free%20Space%20Pathloss%20(ITU%2DR%20MA%20FSPL)%20Model
      ////////////////////////////////////////////////////////////////////////
      // Method 4: Complete Botella-Campos https://ieeexplore.ieee.org/document/9260568 with my fresnel zone tree detection
      ////////////////////////////////////////////////////////////////////////
      // Method 5: Custom parameters for log loss path loss model fitted for following requirements
      // Max range without trees LOS ~ 70m ref- https://ieeexplore.ieee.org/document/7943528
      // Tree attenuation ~ 12 dBm ref - https://ieeexplore.ieee.org/document/9260568
      // Variance ~ 4.4
      // No trees LOS
      // lo = 50
      // fading exponent measured as 3.5 for range ~ 50m
      // Assuming 2 trees LOS
      // lo = 40
      // fading exponent measured as 2.5 for range ~ 50m
      if (treesOnLink == 0)
      {
        // Method 1:
          //pathLoss = CalculateMeadowPathLoss(distance);
        // Method 2:
          //pathLoss = FreeSpacePathLoss(distance);
        // Method 3:
          //pathLoss = CalculateMAITURPathLoss(distance);
        // Method 4:
          //pathLoss = 49.17 - 20 * std::log10(distance);
        // Method 5:
          // envNetworkConfig.setL0(50.0);
          // envNetworkConfig.setFadingExponent(3.5);
          // envNetworkConfig.setVariance(4.4);
          // pathLoss = CalculatePathLossWithVariance(distance, envNetworkConfig);
        // Method 6:
          pathLoss = CalculateTreePathLoss(distance, treesOnLink);
      }

      if (treesOnLink > 0)
      {
        // Method 1:
          //pathLoss = CalculateMeadowPathLoss(distance) + 12 * treesOnLink;
        // Method 2:
          // double vegetationIndex = treesOnLink / (linkModelWidth * distance) * treeRadius * 100 * 2; // VD = TD.D
          // envNetworkConfig.setL0(-0.82 * vegetationIndex + 40.1);
          // envNetworkConfig.setFadingExponent(0.1717 * vegetationIndex + 2.2043);
          // envNetworkConfig.setVariance(4.4);
          // pathLoss = CalculatePathLossWithVariance(distance, envNetworkConfig);
        // Method 3:
          //pathLoss = CalculateMAITURPathLoss(distance) + 11.98 * treesOnLink;
        // Method 4:
          //pathLoss = 49.17 - 20 * std::log10(distance) + 11.98 * treesOnLink;
        // Method 5:
          // envNetworkConfig.setL0(40.0);
          // envNetworkConfig.setFadingExponent(2.5);
          // envNetworkConfig.setVariance(4.4);
          // pathLoss = CalculatePathLossWithVariance(distance, envNetworkConfig) + 12 * treesOnLink;
        // Method 6:
          pathLoss = CalculateTreePathLoss(distance, treesOnLink);
      }

      // Calculate received power
      double rxPower = robotNetworkConfig.getTxPower() - pathLoss;

      // Calculate BER using SNR
      // double ber = QPSKSNRToBER(DbmToPow(rxPower), DbmToPow(envNetworkConfig.getSnrThreshold()));
      double ber = 0.0;

      if (treesOnLink == 0)
      {
          double ber = AWGNQAM64EbNoToBER(DbmToPow(rxPower), DbmToPow(robotNetworkConfig.getAntennaNoiseFloor()));
      }

      if (treesOnLink > 0)
      {
          double ber = RAYLEIGHQAM64SNRToBER(DbmToPow(rxPower), DbmToPow(robotNetworkConfig.getAntennaNoiseFloor()));
      }

      // Calculate PER using BER
      double per = BERToPER(ber, packetSizeInBits);

      // Consolidated debug message for all calculated values
      // igndbg << "Calculations between " << name1 << " and " << name2 << ":\n"
      //         << "  Euclidean Distance: " << distance << "\n"
      //         << "  Number of trees LOS: " << treesOnLink << "\n"
      //         << "  Path Loss: " << pathLoss << "\n"
      //         << "  Rx Power: " << rxPower << "\n"
      //         << "  BER: " << ber << "\n"
      //         << "  PER: " << per << std::endl;

      double SNR = rxPower - robotNetworkConfig.getAntennaNoiseFloor();

      // https://arxiv.org/abs/1301.6644 Thesis shows effective SNR required for 802.11n, 64 QAM is 16.1 dBm
      // However, https://wlanprofessionals.com/mcs-table-and-how-to-use-it/ MCS table gives 25 dBm
      // I am choosing 25 dBm since the environment is not ideal
      // From IEEE 802.11 standard PER should be 8% or 10% to achieve comms however, this is included in the above requirement 
      // so not considering now.
      if (SNR < 25) 
      {
        gz::msgs::Double msgPDR;
        msgPDR.set_data(1.0);

        // Publish to corresponding topics if publishers exist
        if (this->dataPtr->packetDropRatePublishers.count(name1) > 0 &&
            this->dataPtr->packetDropRatePublishers[name1].count(name2) > 0) 
        {
            this->dataPtr->packetDropRatePublishers[name1][name2].Publish(msgPDR);
            //igndbg << "PDR " << name1 << " and " << name2 << ": 1.0" << std::endl;
            //igndbg << "Comms Status:" << name1 << " and " << name2 << " : " << "COMMS NOT AVAILABLE!" << std::endl;
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
            //igndbg << "PDR " << name1 << " and " << name2 << ": 1.0" << std::endl;
            //igndbg << "Comms Status:" << name1 << " and " << name2 << " : " << "COMMS AVAILABLE!" << std::endl;
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
          //igndbg << "PER:" << name1 << " and " << name2 << " : " << per << std::endl;
      }

      if (this->dataPtr->pathLossPublishers.count(name1) > 0 &&
          this->dataPtr->pathLossPublishers[name1].count(name2) > 0) 
      {
          this->dataPtr->pathLossPublishers[name1][name2].Publish(msgPathLoss);
          //igndbg << "Path Loss:" << name1 << " and " << name2 << " : " << pathLoss << std::endl;
      }

      // Publish Rx Signal Strength
      gz::msgs::Double msgRSS;
      msgRSS.set_data(rxPower);

      // Publish to corresponding topics if publishers exist
      if (this->dataPtr->RSSPublishers.count(name1) > 0 &&
          this->dataPtr->RSSPublishers[name1].count(name2) > 0) 
      {
          this->dataPtr->RSSPublishers[name1][name2].Publish(msgRSS);
          //igndbg << "RSS:" << name1 << " and " << name2 << " : " << rxPower << std::endl;
      }

      // Publish Delay
      gz::msgs::Double msgDelay;
      // https://www.researchgate.net/publication/352329230_Development_of_Wi-Fi-Based_Teleoperation_System_for_Forest_Harvester
      double Delay = 2.5; //ms
      msgDelay.set_data(Delay);

      // Publish to corresponding topics if publishers exist
      if (this->dataPtr->DelayPublishers.count(name1) > 0 &&
          this->dataPtr->DelayPublishers[name1].count(name2) > 0) 
      {
          this->dataPtr->DelayPublishers[name1][name2].Publish(msgDelay);
          //igndbg << "Delay (ms):" << name1 << " and " << name2 << " : " << Delay << std::endl;
      }

      // Publish Bandwidth
      gz::msgs::Double msgBandwidth;
      double Bandwidth = 72; //Mbps
      msgBandwidth.set_data(Bandwidth);

      // Publish to corresponding topics if publishers exist
      if (this->dataPtr->BandwidthPublishers.count(name1) > 0 &&
          this->dataPtr->BandwidthPublishers[name1].count(name2) > 0) 
      {
          this->dataPtr->BandwidthPublishers[name1][name2].Publish(msgBandwidth);
          //igndbg << "Bandwidth (Mbps):" << name1 << " and " << name2 << " : " << Bandwidth << std::endl;
      }
    }
  }

}

}  // namespace comms_emulator_helper_system