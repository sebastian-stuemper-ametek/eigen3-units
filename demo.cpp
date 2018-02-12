#include <iostream>
#include "matrix.h"


int main(int argc, char* argv[])
{
	using namespace crobot::math;

	Displacement3D r1 = {
		1.0 * meter,
		2.0 * meter,
		2.0 * meter
	};
	Displacement3D r2 = r1 + r1;
	Velocity3D     v1 = {
		1.0 * meter_per_second,
		2.0 * meter_per_second,
		3.0 * meter_per_second
	};
	Acceleration3D a1 = {
		4.0 * meter_per_second_squared,
		4.0 * meter_per_second_squared,
		5.0 * meter_per_second_squared
	};
	Force3D f1 = {
		1.0 * newton,
		3.0 * newton,
		5.0 * newton
	};
	Time t1 = 6.0  * second;
	AngularAcceleration3D omega = {
		3.0 * rad_per_second_squared,
		2.0 * rad_per_second_squared,
		1.0 * rad_per_second_squared
	};
	AngularVelocity3D theta_dot = {
		3.0 * rad_per_second,
		2.0 * rad_per_second,
		1.0 * rad_per_second
	};
	Angle3D theta = {
		3.13 * rad,
		1.13 * rad,
		0.23 * rad
	};

	std::cout << "\nTime: \n"
		<< t1 << std::endl;
	std::cout << "\nForce 1: \n"
		<< f1 << std::endl;
	std::cout << "\nForce 1 Transpose: \n"
		<< f1.transpose() << std::endl;
	std::cout << "\nDisplacement 1 Transpose: \n"
		<< r1.transpose() << std::endl;
	std::cout << "\nForce 1 Columnwise Square: \n"
		<< f1.cwiseAbs2() << std::endl;
	std::cout << "\nForce 1 Columnwise Square Root: \n"
		<< f1.cwiseSqrt() << std::endl;
	std::cout << "\nf1.dot(r1): \n"
		<< f1.dot(r1) << std::endl;
	std::cout << "\nf1.cross(r1): \n"
		<< f1.cross(r1) << std::endl;
	std::cout << "\nr1.dot(f1): \n"
		<< r1.dot(f1) << std::endl;
	std::cout << "\nr1.cross(f1): \n"
		<< r1.cross(f1) << std::endl;
	std::cout << "\nr1 + r2 = \n"
		<< r1 + r2 << std::endl;
	std::cout << "\nr1 - r2 = \n"
		<< r1 - r2 << std::endl;
	std::cout << "\nr1 * 5.0 = \n"
		<< r1 * 5.0 << std::endl;
	/*std::cout << "\nr1 / 5.0 = \n"
		<< r1 / 5.0 << std::endl;*/
	std::cout << "\nr1.sum() = \n"
		<< r1.sum() << std::endl;
	std::cout << "\nAcceleration 1 (a1) = \n"
		<< a1 << std::endl;
	std::cout << "\nv2 = a1 * t1 + v1 = \n"
		<< a1 * t1 + v1 << std::endl;
	std::cout << "\nv2 * t1 + r1 = \n"
		<< (a1 * t1 + v1) * t1 + r1 << std::endl;
	std::cout << "\nAngularVelocity (theta_dot): \n"
		<< theta_dot << std::endl;
	std::cout << "\nAngularAcceleration (omega): \n"
		<< omega << std::endl;
	std::cout << "\ntheta = \n"
		<< theta << std::endl;
	std::cout << "\nomega * t * t + theta_dot * t + theta = : \n"
		<< omega * t1 * t1 + theta_dot * t1 + theta << std::endl;
	std::cout << "\nr1 * 5.0\n" << r1 * 5.0 << std::endl;
	std::cout << "\n5.0 * r1\n" << 5.0 * r1 << std::endl;
	std::cout << "\nr1 / 5.0\n" << r1 / 5.0 << std::endl;

	std::cout << "\nutil::squared_norm(r1) = \n"
		<< util::squared_norm(r1) << std::endl;
	std::cout << "\nutil::norm(r1) = \n"
		<< util::norm(r1) << std::endl;
	std::cout << "\nutil::normalized(r1) = \n"
        << util::normalized(r1);

	return 0;
}