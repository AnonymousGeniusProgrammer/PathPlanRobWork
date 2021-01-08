//#define _LOADWC
//#define _ROTATION_MATRIX
//#define _RPY
//#define _EAA
//#define _QUATERNION


#ifdef _LOADWC

#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
using namespace rw::models;
using rw::loaders::WorkCellLoader;

void printDeviceNames(const WorkCell& workcell)
{
	std::cout << "Workcell " << workcell << " contains devices:" << std::endl;
	for (const Device::CPtr device : workcell.getDevices()) {
		std::cout << "- " << device->getName() << std::endl;
	}
}
#endif // _LOADWC

#ifdef _ROTATION_MATRIX

#include <rw/math/Rotation3D.hpp>
using rw::math::Rotation3D;

void egInvRot()
{
	Rotation3D<> rotd = Rotation3D<>(1, 0, 0, 0, 0, -1, 0, 1, 0);
	Rotation3D<float> rotf = Rotation3D<float>(1, 0, 0, 0, 0, -1, 0, 1, 0);

	std::cout << "Rotation double:" << std::endl << rotd << std::endl;
	std::cout << "Rotation float:" << std::endl << rotf << std::endl;
	std::cout << "Rotation inverse:" << std::endl << inverse(rotd) << std::endl;
	std::cout << "Identity:" << std::endl << rotd * inverse(rotd) << std::endl;
}

#endif // _ROT

#ifdef _RPY

#include <rw/math/Constants.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>

using namespace rw::math;

void egRPY()
{
	const RPY<> rpy = RPY<>(Pi, Pi / 2, 0);
	std::cout << "RPY: " << rpy << std::endl;
	const Rotation3D<> rotationFromRPY = rpy.toRotation3D();
	std::cout << "Rotation from RPY: " << rotationFromRPY << std::endl;

	const Rotation3D<> rot = Rotation3D<>(-1, 0, 0, 0, 0, 1, 0, 1, 0);
	std::cout << "Rotation: " << rot << std::endl;
	const RPY<> rpyFromRotation = RPY<>(rot);
	std::cout << "RPY from Rotation: " << rpyFromRotation << std::endl;
}
#endif //_RPY

#ifdef _EAA
#include <rw/math/Constants.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Rotation3D.hpp>

using namespace rw::math;

void egEAA()
{
	const EAA<> eaa = EAA<>(sqrt(2) / 2 * Pi, sqrt(2) / 2 * Pi, 0);
	std::cout << "EAA: " << eaa << std::endl;
	std::cout << " angle: " << eaa.angle() << std::endl;
	std::cout << " axis: " << eaa.axis() << std::endl;
	const Rotation3D<> rotationFromEAA = eaa.toRotation3D();
	std::cout << "Rotation from EAA: " << rotationFromEAA << std::endl;

	const Rotation3D<> rot = Rotation3D<>(-1, 0, 0, 0, 0, 1, 0, 1, 0);
	std::cout << "Rotation: " << rot << std::endl;
	const EAA<> eaaFromRotation = EAA<>(rot);
	std::cout << "EAA from Rotation: " << eaaFromRotation << std::endl;
	std::cout << " angle: " << eaaFromRotation.angle() << std::endl;
	std::cout << " axis: " << eaaFromRotation.axis() << std::endl;
}

#endif // _EAA

#ifdef _QUATERNION
#include <rw/math/Quaternion.hpp>
#include <rw/math/Rotation3D.hpp>

using namespace rw::math;

void egQuaternion()
{
	const Quaternion<> quat = Quaternion<>(sqrt(2) / 2, sqrt(2) / 2, 0, 0);
	std::cout << "Quaternion: " << quat << std::endl;
	const Rotation3D<> rotationFromQuat = quat.toRotation3D();
	std::cout << "Rotation from Quaternion: " << rotationFromQuat << std::endl;

	const Rotation3D<> rot = Rotation3D<>(-1, 0, 0, 0, 0, 1, 0, 1, 0);
	std::cout << "Rotation: " << rot << std::endl;
	const Quaternion<> quatFromRotation = Quaternion<>(rot);
	std::cout << "Quaternion from Rotation: " << quatFromRotation << std::endl;
}
#endif // _QUATERNION


int main(int argc, char** argv)
{
#ifdef _LOADWC
	if (argc != 2) {
		std::cout << "Usage: " << argv[0] << " <path/to/RobWorkData>" << std::endl;
		exit(1);
	}
	const std::string path = argv[1];

	const WorkCell::Ptr wc = WorkCellLoader::Factory::load(path + WC_FILE);
	if (wc.isNull())
		RW_THROW("WorkCell could not be loaded.");

	std::cout << "Workcell " << wc->getName();
	std::cout << " successfully loaded." << std::endl;
	printDeviceNames(*wc);

#endif // LOADWC

#ifdef _ROTATION_MATRIX
	egInvRot();
#endif // _ROTATION_MATRIX

#ifdef _EAA
	egEAA();
#endif // _EAA

#ifdef _RPY
	egRPY();
#endif // _RPY

#ifdef _QUATERNION
	egQuaternion();
#endif // _QUATERNION
}