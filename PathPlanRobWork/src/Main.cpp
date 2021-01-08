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

#include <fstream>
//#include <iostream>
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
#define SQT_2_2 sqrt(2) / 2

struct FCwp
{
	double px;
	double py;
	double pz;
	double q0;
	double q1;
	double q2;
	double q3;
};


void ofsStates(const Device::Ptr device, const std::vector<State> states, std::ofstream* ofs);

void ofsResults(QPath result, std::ofstream* ofs);

inline const Transform3D<> wpToTrans3D(FCwp* wp);

void setFCwp(FCwp* wp, double px, double py, double pz, double q0, double q1, double q2, double q3);

void plan(QToTPlanner::Ptr planner, Q beg, const Transform3D<> end, QPath* result);

void planForWps(QToTPlanner::Ptr planner, std::vector< Q >& begs, QPath * result, QPath * wp_in_csp, std::vector<FCwp> wps);

void genLineWps(std::vector<FCwp>* wps, double start, double end, int intpol);

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

	const QToTPlanner::Ptr planner = SBLPlanner::makeQToTPlanner(sblsetup, ik_cfree);

	// Set up base trajectory 
	const int dim = 9;
	const Q cinit(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	ProximityData pdata;
	State state = defState;
	device->setQ(cinit, state);
	if (collisionDetector->inCollision(state, pdata))
		RW_THROW("Initial configuration in collision! can not plan a path.");
	
	// starting points for planning iterations
	std::vector<Q> begs;
	std::vector<Q> begs_1;
	Q init = cinit;
	begs.push_back(init);

	// ending opints for planning iterations
	std::vector<FCwp> wps;
	std::vector<FCwp> wps_1;

	// resulting path in configuration space
	QPath result;
	QPath wp_in_csp;
	//QPath result1;
	std::ofstream qout;
	
	genLineWps(&wps, 0.55, 1, 5);
	planForWps(planner, begs, &result, &wp_in_csp, wps);
	//const size_t ind_end = result.size() - 1;
	//begs_1.push_back(result[result.size() - 1]);
	//std::cout << "get new starting point for next segment planning from end config in previous segment." << std::endl;

	//genLineWps(&wps_1, 1, 1, 1);
	//std::cout << "generating new wps for next segment." << std::endl;

	// Adding new constraints in this segment!
	/*Device::QBox bounds = Device::QBox();*/
	
	//planForWps(planner, begs_1, &result, wps_1);
	
	//const size_t ind_end_1 = result.size() - 1;
	//QPath c_end;
	//c_end.push_back(result[ind_end]);
	//c_end.push_back(result[ind_end_1]);

	const std::vector<State> states = Models::getStatePath(*device, result, state);

	const std::vector<State> states_1 = Models::getStatePath(*device, wp_in_csp, state);
	ofsResults(result, &qout);
	std::cout << "I am here after ofsStates" << std::endl;
	PathLoader::storeVelocityTimedStatePath(*wc, states, "../../out/ex-path-planning.rwplay");
	//PathLoader::storeVelocityTimedStatePath(*wc, states_1, "../../out/ex-path-planning.rwplay");
	std::cout << "I am here after sotreVelocityTimedStatePath" << std::endl;

	return 0;
}

// only for testing strategies
void genLineWps(std::vector<FCwp>* wps, double start, double end, int intpol)
{
	double inc = (end - start) / intpol;
	for (int i = 0; i < intpol; ++i)
	{
		FCwp wp;
		double next = start + inc * i;
		if (next < end)
		{
			setFCwp(&wp, 0.1, -0.1, start + inc * i, SQT_2_2, 0, 0, -SQT_2_2);
			wps->push_back(wp);
		}
		else
		{
			setFCwp(&wp, 0.1, -0.1, end, SQT_2_2, 0, 0, -SQT_2_2);
			wps->push_back(wp);
		}

	}
}

void planForWps(QToTPlanner::Ptr planner, std::vector< Q >& begs, QPath * result, QPath * wp_in_csp, std::vector<FCwp> wps)
{
	for (int i = 0; i < wps.size(); ++i)
	{
		const Transform3D<> end = wpToTrans3D(&wps[i]);
		plan(planner, begs[i], end, result);
		begs.push_back(result->at(result->size() - 1));
		wp_in_csp->push_back(result->at(result->size() - 1));
	}
}

void plan(QToTPlanner::Ptr planner, Q beg, const Transform3D<> end, QPath * result)
{
	std::cout << beg << std::endl;
	const Q cbeg = beg;
	/*std::cout << cbeg << std::endl;*/
	if (planner->query(cbeg, end, *result)) {
		std::cout << "Planned path with " << result->size();
		std::cout << " configurations" << std::endl;
	}

}

void setFCwp(FCwp* wp, double px, double py, double pz, double q0, double q1, double q2, double q3)
{
	wp->px = px;
	wp->py = py;
	wp->pz = pz;
	wp->q0 = q0;
	wp->q1 = q1;
	wp->q2 = q2;
	wp->q3 = q3;
}

const Transform3D<> wpToTrans3D(FCwp* wp)
{
	const Quaternion<> quat = Quaternion<>(wp->q0, wp->q1, wp->q2, wp->q3);
	const Vector3D<> trans = Vector3D<>(wp->px, wp->py, wp->pz);
	const Transform3D<> end = Transform3D<>(trans, quat.toRotation3D());
	return end;
}

void ofsStates(const Device::Ptr device, std::vector<State> states, std::ofstream* ofs) 
{
	if (!ofs->is_open()) {
		ofs->open("../../out/qout.txt");
	}
	for (int i = 0; i < states.size(); i++) {
		*ofs << device->getQ(states[i]) << std::endl;
	}
	ofs->close();
}

void ofsResults(QPath result, std::ofstream* ofs)
{
	if (!ofs->is_open()) {
		ofs->open("../../out/qout.txt");
	}
	for (int i = 0; i < result.size(); i++) {
		*ofs << result[i] << std::endl;
	}
	ofs->close();
}
