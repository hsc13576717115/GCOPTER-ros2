#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <stdexcept>
#include <cstring>
#include <sstream>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    Config(rclcpp::Node &nh_priv)
    {
        mapTopic = nh_priv.declare_parameter("MapTopic", std::string("/voxel_map"));
        targetTopic = nh_priv.declare_parameter("TargetTopic", std::string("/move_base_simple/goal"));
        dilateRadius = nh_priv.declare_parameter("DilateRadius", 0.5);
        voxelWidth = nh_priv.declare_parameter("VoxelWidth", 0.25);
        mapBound = nh_priv.declare_parameter("MapBound", std::vector<double>{-25.0,25.0,-25.0,25.0,0.0,5.0});
        timeoutRRT = nh_priv.declare_parameter("TimeoutRRT", 0.5);
        maxVelMag = nh_priv.declare_parameter("MaxVelMag", 4.0);
        maxBdrMag = nh_priv.declare_parameter("MaxBdrMag", 2.1);
        maxTiltAngle = nh_priv.declare_parameter("MaxTiltAngle", 1.05);
        minThrust = nh_priv.declare_parameter("MinThrust", 2.0);
        maxThrust = nh_priv.declare_parameter("MaxThrust", 12.0);
        vehicleMass = nh_priv.declare_parameter("VehicleMass", 0.61);
        gravAcc = nh_priv.declare_parameter("GravAcc", 9.8);
        horizDrag = nh_priv.declare_parameter("HorizDrag", 0.70);
        vertDrag = nh_priv.declare_parameter("VertDrag", 0.80);
        parasDrag = nh_priv.declare_parameter("ParasDrag", 0.01);
        speedEps = nh_priv.declare_parameter("SpeedEps", 0.0001);
        weightT = nh_priv.declare_parameter("WeightT", 20.0);
        chiVec = nh_priv.declare_parameter("ChiVec", std::vector<double>{1.e4,1.e4,1.e4,1.e4,1.e5});
        smoothingEps = nh_priv.declare_parameter("SmoothingEps", 1.e-2);
        integralIntervs = nh_priv.declare_parameter("IntegralIntervs", 16);
        relCostTol = nh_priv.declare_parameter("RelCostTol", 1.e-5);
        auto finite = [](const std::vector<double> &v) { return std::all_of(v.begin(), v.end(), [](double x){return std::isfinite(x);}); };
        if (mapBound.size()!=6 || chiVec.size()!=5 || !finite(mapBound) || !finite(chiVec) ||
            !finite({dilateRadius,voxelWidth,timeoutRRT,maxVelMag,maxBdrMag,maxTiltAngle,minThrust,maxThrust,
                     vehicleMass,gravAcc,horizDrag,vertDrag,parasDrag,speedEps,weightT,smoothingEps,relCostTol}) ||
            voxelWidth<=0 || dilateRadius<0 || timeoutRRT<=0 || maxVelMag<=0 || maxBdrMag<=0 ||
            maxTiltAngle<=0 || maxTiltAngle>=3.141592653589793 || minThrust<0 || maxThrust<=minThrust ||
            vehicleMass<=0 || gravAcc<=0 || horizDrag<0 || vertDrag<0 || parasDrag<0 || speedEps<=0 ||
            weightT<=0 || smoothingEps<=0 || integralIntervs<1 || relCostTol<=0 ||
            std::any_of(chiVec.begin(),chiVec.end(),[](double x){return x<=0;}))
            throw std::invalid_argument("Invalid planner parameters: check bounds, weights and physical limits");
        double cells=1;
        for(int i=0;i<3;++i) {
            const double n=(mapBound[2*i+1]-mapBound[2*i])/voxelWidth;
            if(n<1 || std::abs(n-std::round(n))>1.e-6) throw std::invalid_argument("MapBound spans must be positive multiples of VoxelWidth");
            cells*=n;
        }
        if(cells>20000000 || mapBound[5]-mapBound[4]<=2*dilateRadius || mapTopic.empty() || targetTopic.empty())
            throw std::invalid_argument("Invalid map size, clearance or topic");

    }
};

class GlobalPlanner : public rclcpp::Node
{
private:
    Config config;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub;
    rclcpp::TimerBase::SharedPtr timer;
    bool playing = false;
    int sequence = 0;
    void status(const std::string &text) {
        std_msgs::msg::String msg; msg.data=text; statusPub->publish(msg);
        RCLCPP_INFO(get_logger(), "%s", text.c_str());
    }
    void clearTrajectory() { playing=false; traj.clear(); visualizer.clear(); }


    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    voxel_map::VoxelMap planningMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    double trajStamp;

public:
    GlobalPlanner() : Node("global_planning_node"), config(*this), mapInitialized(false), visualizer(*this) {
        const auto seed=declare_parameter<int>("PlanningSeed", 42);
        if(seed<1) throw std::invalid_argument("PlanningSeed must be positive");
        ompl::RNG::setSeed(static_cast<unsigned int>(seed));
        statusPub=create_publisher<std_msgs::msg::String>("/planner/status",rclcpp::QoS(10).reliable().transient_local());
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        mapSub=create_subscription<sensor_msgs::msg::PointCloud2>(config.mapTopic, rclcpp::QoS(1).reliable().transient_local(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){mapCallBack(msg);});
        targetSub=create_subscription<geometry_msgs::msg::PoseStamped>(config.targetTopic,10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){targetCallBack(msg);});
        timer=create_wall_timer(std::chrono::milliseconds(10),[this](){process();});
        status("WAITING_FOR_MAP");
    }

    void mapCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        if(mapInitialized) return;
        if(!msg->header.frame_id.empty() && msg->header.frame_id!="odom") { status("INVALID_MAP frame must be odom"); return; }
        // Validate the entire layout before modifying the static map. Respect offsets, row padding and endian order.
        const std::size_t count=std::size_t(msg->width)*msg->height;
        if(count==0) {
            if(!msg->data.empty()) {status("INVALID_MAP inconsistent empty cloud"); return;}
        } else {
            if(msg->point_step==0 || std::size_t(msg->row_step)<std::size_t(msg->width)*msg->point_step ||
               std::size_t(msg->row_step)*msg->height>msg->data.size()) {status("INVALID_MAP invalid strides"); return;}
            std::array<uint32_t,3> offsets{};
            const char *names[]={"x","y","z"};
            for(int axis=0;axis<3;++axis) {
                auto field=std::find_if(msg->fields.begin(),msg->fields.end(),[&](const auto &f){return f.name==names[axis];});
                if(field==msg->fields.end() || field->datatype!=sensor_msgs::msg::PointField::FLOAT32 || field->count!=1 ||
                   std::size_t(field->offset)+sizeof(float)>msg->point_step) {status("INVALID_MAP expected float32 x/y/z fields"); return;}
                offsets[axis]=field->offset;
            }
            const uint16_t one=1; const bool hostBig=*(reinterpret_cast<const uint8_t*>(&one))==0;
            std::size_t valid=0;
            for(uint32_t row=0;row<msg->height;++row) for(uint32_t col=0;col<msg->width;++col) {
                Eigen::Vector3d point;
                for(int axis=0;axis<3;++axis) {
                    const auto *ptr=msg->data.data()+std::size_t(row)*msg->row_step+std::size_t(col)*msg->point_step+offsets[axis];
                    std::array<uint8_t,4> bytes; std::copy_n(ptr,4,bytes.begin());
                    if(hostBig!=msg->is_bigendian) std::reverse(bytes.begin(),bytes.end());
                    float v; std::memcpy(&v,bytes.data(),4); point(axis)=v;
                }
                if(point.allFinite()) {voxelMap.setOccupied(point); ++valid;}
            }
            if(valid==0) {status("INVALID_MAP no finite points"); return;}
        }
        // FIRI sees voxel centers rather than voxel volumes. An extra planning voxel
        // provides geometric clearance; validate the result against the requested dilation.
        planningMap=voxelMap;
        planningMap.dilate(int(std::ceil(config.dilateRadius/config.voxelWidth))+1);
        voxelMap.dilate(int(std::ceil(config.dilateRadius/config.voxelWidth)));
        mapInitialized=true; status("MAP_READY");
    }

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            status("PLANNING " + std::to_string(++sequence));
            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &planningMap, config.timeoutRRT,
                                                   route);
            if(route.size()<2) {status("FAILED path search"); return;}
            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            planningMap.getSurf(pc);

            sfc_gen::convexCover(route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 3.0,
                                 hPolys);
            sfc_gen::shortCut(hPolys);
            if(hPolys.empty()) {status("FAILED corridor"); return;}

            if (route.size() > 1)
            {
                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();

                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    clearTrajectory(); status("FAILED optimizer setup"); return;
                }

                if (!std::isfinite(gcopter.optimize(traj, config.relCostTol)))
                {
                    clearTrajectory(); status("FAILED optimizer"); return;
                }

                if (traj.getPieceNum() > 0)
                {
                    if(!validateTrajectory(startGoal.back())) {traj.clear(); visualizer.clear(); status("FAILED trajectory validation"); return;}
                    trajStamp = now().seconds(); playing=true;
                    visualizer.visualize(traj, route);
                    status("SUCCESS " + std::to_string(sequence) + " duration=" + std::to_string(traj.getTotalDuration()));
                }
                else {clearTrajectory(); status("FAILED empty optimized trajectory");}
            }
        }
    }

    void targetCallBack(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
        if(!mapInitialized) {status("REJECTED map not ready"); return;}
        if(!msg->header.frame_id.empty() && msg->header.frame_id!="odom") {status("REJECTED frame must be odom"); return;}
        const auto &p=msg->pose;
        if(!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.orientation.z) ||
           std::abs(p.orientation.z)>1.0) {status("REJECTED invalid pose"); return;}
        const double z=config.mapBound[4]+config.dilateRadius+std::abs(p.orientation.z)*
            (config.mapBound[5]-config.mapBound[4]-2*config.dilateRadius);
        const Eigen::Vector3d goal(p.position.x,p.position.y,z);
        if(planningMap.query(goal)) {status("REJECTED occupied, within planning clearance or outside map"); return;}
        if(startGoal.size()>=2) {startGoal.clear(); clearTrajectory();}
        if(!startGoal.empty() && (goal-startGoal.front()).norm()<1.e-6) {status("REJECTED identical start and goal"); return;}
        visualizer.visualizeStartGoal(goal,0.5,startGoal.size()); startGoal.emplace_back(goal);
        if(startGoal.size()==1) status("START_ACCEPTED");
        else {
            try {plan();}
            catch(const std::exception &e) {clearTrajectory(); status(std::string("FAILED ")+e.what());}
        }
    }

    bool validateTrajectory(const Eigen::Vector3d &goal) {
        const double duration=traj.getTotalDuration();
        if(!std::isfinite(duration) || duration<=0 || !(traj.getDurations().array()>0).all()) return false;
        flatness::FlatnessMap fm;
        fm.reset(config.vehicleMass,config.gravAcc,config.horizDrag,config.vertDrag,config.parasDrag,config.speedEps);
        Eigen::Matrix<double,5,1> violations=Eigen::Matrix<double,5,1>::Zero();
        // Discrete numerical validation, not a continuous-time safety certificate.
        const double dt=std::min(0.01,config.voxelWidth/(2*config.maxVelMag));
        for(double t=0;;t=std::min(t+dt,duration)) {
            const auto pos=traj.getPos(t), vel=traj.getVel(t), acc=traj.getAcc(t), jer=traj.getJer(t);
            if(!pos.allFinite()||!vel.allFinite()||!acc.allFinite()||!jer.allFinite()) {status("INVALID_TRAJECTORY nonfinite state"); return false;}
            if(voxelMap.query(pos)) {std::ostringstream msg;msg<<"INVALID_TRAJECTORY occupied sample t="<<t<<" p="<<pos.transpose();status(msg.str());return false;}
            double thrust; Eigen::Vector4d quat; Eigen::Vector3d omega;
            fm.forward(vel,acc,jer,0,0,thrust,quat,omega);
            if(!std::isfinite(thrust)||!quat.allFinite()||!omega.allFinite()) return false;
            const double tilt=std::acos(std::clamp(1-2*(quat(1)*quat(1)+quat(2)*quat(2)),-1.0,1.0));
            Eigen::Matrix<double,5,1> v;
            v << vel.norm()-config.maxVelMag,omega.norm()-config.maxBdrMag,tilt-config.maxTiltAngle,
                 config.minThrust-thrust,thrust-config.maxThrust;
            violations=violations.cwiseMax(v);
            if(t>=duration) break;
        }
        const double error=(traj.getPos(duration)-goal).norm();
        if(!std::isfinite(error)||error>0.001) return false;
        std::ostringstream out; out<<"VALIDATED duration="<<duration<<" endpoint_error="<<error<<" sample_dt="<<dt
            <<" violations[v,omega,tilt,thrust_min,thrust_max]="<<violations.transpose(); status(out.str());
        return true;
    }

    inline void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (playing && traj.getPieceNum() > 0)
        {
            const double elapsed=now().seconds()-trajStamp;
            const double delta=std::clamp(elapsed,0.0,traj.getTotalDuration());
            if (delta >= 0.0)
            {
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                0.0, 0.0,
                                thr, quat, omg);
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();
                double tiltangle = acos(std::clamp(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)), -1.0, 1.0));
                std_msgs::msg::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub->publish(speedMsg);
                visualizer.thrPub->publish(thrMsg);
                visualizer.tiltPub->publish(tiltMsg);
                visualizer.bdrPub->publish(bdrMsg);

                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
                if(elapsed>=traj.getTotalDuration()) {playing=false;status("COMPLETED " + std::to_string(sequence));}
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {rclcpp::spin(std::make_shared<GlobalPlanner>());}
    catch(const std::exception &e) {RCLCPP_ERROR(rclcpp::get_logger("gcopter"),"%s",e.what());rclcpp::shutdown();return 1;}
    rclcpp::shutdown();return 0;
}
