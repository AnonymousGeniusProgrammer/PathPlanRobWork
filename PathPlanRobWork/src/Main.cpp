#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/CompositeDevice.hpp>
#include <rw/models/Device.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/ProximityData.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rw/models/Models.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/pathplanning/QToTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rw/pathplanning/QIKSampler.hpp>
// #include <rw/pathplanning/QConstraint.hpp>
#include <direct.h>
using rw::common::ownedPtr;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;
using rw::math::Q;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::pathplanning;
using rw::trajectory::QPath;
using rwlibs::pathplanners::SBLPlanner;
using rwlibs::proximitystrategies::ProximityStrategyFactory;

using rw::loaders::PathLoader;
using rwlibs::pathplanners::SBLSetup;
using rw::math::Transform3D;
using rw::math::Quaternion;
using rw::math::Vector3D;
using rw::math::Rotation3D;
using rw::math::Rotation3DVector;

#define WC_FILE "/scenes/SinglePA10Demo/SinglePA10DemoGantry.wc.xml"

int main(int argc, char** argv)
{
	if (argc != 2) {
		std::cout << "Usage: " << argv[0] << " <path/to/RobWorkData>" << std::endl;
		exit(1);
	}
	const std::string path = argv[1];

	const WorkCell::Ptr wc = WorkCellLoader::Factory::load(path + WC_FILE);

	if (wc.isNull())
		RW_THROW("WorkCell could not be loaded.");
	const Device::Ptr gantry = wc->findDevice("Gantry");
	const Device::Ptr pa10 = wc->findDevice("PA10");
	if (gantry.isNull())
		RW_THROW("Gantry device could not be found.");
	if (pa10.isNull())
		RW_THROW("PA10 device could not be found.");

	const State defState = wc->getDefaultState();
	const Device::Ptr device = ownedPtr(new CompositeDevice(
		gantry->getBase(), wc->getDevices(), pa10->getEnd(), "Composite", defState));

	const CollisionStrategy::Ptr cdstrategy =
		ProximityStrategyFactory::makeCollisionStrategy("PQP");
	if (cdstrategy.isNull())
		RW_THROW("PQP Collision Strategy could not be found.");
	const CollisionDetector::Ptr collisionDetector =
		ownedPtr(new CollisionDetector(wc, cdstrategy));

	const QConstraint::Ptr constraint = QConstraint::make(collisionDetector, device, defState);
	const QEdgeConstraintIncremental::Ptr edgeconstraint = QEdgeConstraintIncremental::makeDefault(
		constraint, device);

	const SBLSetup sblsetup = SBLSetup::make(constraint, edgeconstraint, device);
	const QIKSampler::Ptr ik_any = QIKSampler::make(device, defState, NULL, NULL, 25);
	const QIKSampler::Ptr ik_cfree = QIKSampler::makeConstrained(ik_any, constraint, 25);

	// const PlannerConstraint con    = PlannerConstraint::make (collisionDetector, device, defState);
	// const QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner (con, device);
	const QToTPlanner::Ptr planner = SBLPlanner::makeQToTPlanner(sblsetup, ik_cfree);

	// const Q beg (9, -0.67, -0.79, 2.8, -0.02, -1.01, -0.26, -0.77, -1.01, 0);
	int dim = 9;
	const Q beg(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	// const Q end (dim, 0, 0, 0.62, 0, 0, 0, 0, 0, 0);
	const Quaternion<> quat = Quaternion<>(sqrt(2) / 2, 0.0, 0.0, -sqrt(2) / 2);
	// const Rotation3D<> rot = Rotation3D<>(quat);
	const Vector3D<> trans = Vector3D<>(0.1, -0.1, 0.55);
	const Transform3D<> end = Transform3D<>(trans, quat.toRotation3D());

	ProximityData pdata;
	State state = defState;
	device->setQ(beg, state);
	if (collisionDetector->inCollision(state, pdata))
		RW_THROW("Initial configuration in collision! can not plan a path.");
	// device->setQ (end, state);
	// if (collisionDetector->inCollision (state, pdata))
	//     RW_THROW ("Final configuration in collision! can not plan a path.");

	QPath result;
	if (planner->query(beg, end, result)) {
		std::cout << "Planned path with " << result.size();
		std::cout << " configurations" << std::endl;
	}

	// const Q beg1 = device->getQ(state);
	// const Quaternion<> quat1 = Quaternion<> (sqrt(2)/2, 0.0, 0.0, -sqrt(2)/2);
	// // const Rotation3D<> rot1 = Rotation3D<>(quat1);
	// const Vector3D<> trans1 = Vector3D<> (0.1, -0.1, 0.8);
	// const Transform3D<> end1 = Transform3D<>(trans1, quat1.toRotation3D());

	// planner->query (beg1, end1, result);

	const std::vector<State> states = Models::getStatePath(*device, result, state);
	//int state_len = sizeof(states) / sizeof(states[0]); //wrong way to get size of states
	const std::vector<double> config = device->getQ(states[1]).toStdVector();
	for (int i = 0; i < 9; i++)
	{
		std::cout << config[i] << " ";
	}
	std::cout << states.size() << std::endl;
	std::cout << config.size() << std::endl; //9
	//std::cout << state_len; //0
	PathLoader::storeVelocityTimedStatePath(
		*wc, states, "../../out/ex-path-planning.rwplay");
}