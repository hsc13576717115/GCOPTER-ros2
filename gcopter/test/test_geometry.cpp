#include <gtest/gtest.h>
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/gcopter.hpp"

TEST(Voxels, BoundariesAndZeroDilation) {
    voxel_map::VoxelMap map({8,8,8},{0,0,0},0.25);
    EXPECT_TRUE(map.query(Eigen::Vector3d(-0.001,0.1,0.1)));
    map.setOccupied(Eigen::Vector3d(-0.001,0.1,0.1));
    EXPECT_FALSE(map.query(Eigen::Vector3d(0.1,0.1,0.1)));
    EXPECT_TRUE(map.query(Eigen::Vector3d(2,0.1,0.1)));
    EXPECT_TRUE(map.query(Eigen::Vector3d(NAN,0,0)));
    map.setOccupied(Eigen::Vector3d(0.1,0.1,0.1)); map.dilate(0);
    std::vector<Eigen::Vector3d> surf; map.getSurf(surf);
    ASSERT_EQ(surf.size(),1u); EXPECT_NEAR(surf[0].x(),0.125,1.e-12);
}
TEST(Corridor, EmptyAndRepeatedInput) {
    std::vector<Eigen::MatrixX4d> polys;
    sfc_gen::convexCover({}, {}, {-5,-5,0},{5,5,3},7,3,polys);
    sfc_gen::shortCut(polys); EXPECT_TRUE(polys.empty());
    sfc_gen::convexCover({{-2,0,1.5},{-2,0,1.5},{2,0,1.5}}, {}, {-5,-5,0},{5,5,3},7,3,polys);
    EXPECT_FALSE(polys.empty());
    for(const auto &p:polys) EXPECT_TRUE(p.allFinite());
}
TEST(Planning, ExactPathAroundObstacleAndMinimumJerkTrajectory) {
    ompl::RNG::setSeed(42);
    voxel_map::VoxelMap map({40,40,12},{-5,-5,0},0.25);
    for(int z=0;z<12;++z) for(int x=19;x<=20;++x) for(int y=17;y<=22;++y) map.setOccupied(Eigen::Vector3i(x,y,z));
    auto planningMap=map; planningMap.dilate(2);
    map.dilate(1);
    std::vector<Eigen::Vector3d> route,surface;
    const Eigen::Vector3d a(-3,0,1.5),b(3,0,1.5);
    ASSERT_TRUE(std::isfinite(sfc_gen::planPath(a,b,map.getOrigin(),map.getCorner(),&planningMap,1.0,route)));
    ASSERT_GT(route.size(),1u); EXPECT_LT((route.back()-b).norm(),1.e-6);
    planningMap.getSurf(surface); std::vector<Eigen::MatrixX4d> polys;
    sfc_gen::convexCover(route,surface,map.getOrigin(),map.getCorner(),7,3,polys); sfc_gen::shortCut(polys);
    ASSERT_FALSE(polys.empty());
    Eigen::Matrix3d initial=Eigen::Matrix3d::Zero(), final=initial;initial.col(0)=a;final.col(0)=b;
    Eigen::VectorXd limits(5),weights(5),physics(6);
    limits<<4,2.1,1.05,2,12;weights<<1.e4,1.e4,1.e4,1.e4,1.e5;physics<<.61,9.8,.7,.8,.01,.0001;
    gcopter::GCOPTER_PolytopeSFC optimizer;
    ASSERT_TRUE(optimizer.setup(20,initial,final,polys,INFINITY,.01,16,limits,weights,physics));
    Trajectory<5> traj;ASSERT_TRUE(std::isfinite(optimizer.optimize(traj,1.e-5)));
    ASSERT_GT(traj.getTotalDuration(),0);EXPECT_LT((traj.getPos(traj.getTotalDuration())-b).norm(),.001);
    for(double t=0;t<traj.getTotalDuration();t+=.01) {
        ASSERT_TRUE(traj.getPos(t).allFinite()); ASSERT_FALSE(map.query(traj.getPos(t)));
    }
    EXPECT_LT(traj.getVel(traj.getTotalDuration()).norm(),1.e-5);
    EXPECT_LT(traj.getAcc(traj.getTotalDuration()).norm(),1.e-5);
}
