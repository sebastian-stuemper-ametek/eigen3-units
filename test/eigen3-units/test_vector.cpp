#include <gtest/gtest.h>
#include "matrix.h"

using namespace crobot::math;

TEST(Vector, AddOperator)
{
	Displacement3D d1 = {1.0 * meter, 2.0 * meter, 3.0 * meter};
	Displacement3D d2 = {4.0 * meter, 5.0 * meter, 6.0 * meter};
	Displacement3D d  = d1 + d2;
	Displacement3D d_expect = {5.0 * meter, 7.0 * meter, 9.0 * meter};

	ASSERT_TRUE(util::is_approx(d , d_expect, 0.01));
}

TEST(Vector, SubOperator)
{
	Displacement3D d1 = {1.0 * meter, 2.0 * meter, 3.0 * meter};
	Displacement3D d2 = {4.0 * meter, 5.0 * meter, 6.0 * meter};
	Displacement3D d  = d1 - d2;
	Displacement3D d_expect = {-3.0 * meter, -3.0 * meter, -3.0 * meter};

	ASSERT_TRUE(util::is_approx(d , d_expect, 0.01));
}

TEST(Vector, AddEqualOperator)
{
	Displacement3D d1 = {1.0 * meter, 2.0 * meter, 3.0 * meter};
	Displacement3D d2 = {4.0 * meter, 5.0 * meter, 6.0 * meter};
	d1 += d2;
	Displacement3D d_expect = {5.0 * meter, 7.0 * meter, 9.0 * meter};

	ASSERT_TRUE(util::is_approx(d1 , d_expect, 0.01));
}

TEST(Vector, SubEqualOperator)
{
	Displacement3D d1 = {1.0 * meter, 2.0 * meter, 3.0 * meter};
	Displacement3D d2 = {4.0 * meter, 5.0 * meter, 6.0 * meter};
	d1 -= d2;
	Displacement3D d_expect = {-3.0 * meter, -3.0 * meter, -3.0 * meter};

	ASSERT_TRUE(util::is_approx(d1 , d_expect, 0.01));
}

TEST(Vector, CrossProduct)
{
	Force3D        force   = {1.0 * newton, 2.0 * newton, 2.0 * newton};
	Displacement3D disp    = {1.0 * meter,  3.0 * meter,  5.0 * meter};
	Torque3D       torque  = disp.cross(force);
	Torque3D torque_expect = {-4.0 * newton_meter, 3.0 * newton_meter, -1.0 * newton_meter};

	ASSERT_TRUE(util::is_approx(torque, torque_expect, 0.01));
}


// TEST(Vector, CrossProduct)
// {
// 	Force3D        force   = {1.0 * newton, 2.0 * newton, 2.0 * newton};
// 	Displacement3D disp    = {1.0 * meter,  3.0 * meter,  5.0 * meter};
// 	Torque3D       torque  = disp.cross3(force);
// 	Torque3D torque_expect = {-4.0 * newton_meter, 3.0 * newton_meter, -1.0 * newton_meter};

// 	ASSERT_TRUE(util::is_approx(torque, torque_expect, 0.01));
// }

TEST(Vector, DotProduct)
{
	Force3D        force   = {1.0 * newton, 2.0 * newton, 8.0 * newton};
	Displacement3D disp    = {1.0 * meter,  3.0 * meter,  5.0 * meter};
	Energy         energy  = disp.dot(force);
	Energy  energy_expect  = 47.0 * joule;

	ASSERT_TRUE(util::is_approx(energy, energy_expect, 0.01));
}

TEST(Vector, ColWiseSquare)
{
	AngularAcceleration3D  omega_dot = {
		1.0 * rad_per_second_squared,
		4.0 * rad_per_second_squared,
		9.0 * rad_per_second_squared};
	AngularVelocity3D      omega = {
		1.0 * rad_per_second,
		2.0 * rad_per_second,
		3.0 * rad_per_second};
	AngularAcceleration3D result = omega.cwiseAbs2();
	ASSERT_TRUE(util::is_approx(result, omega_dot, 0.01));
}

TEST(Vector, ColWiseSqrt)
{
	AngularAcceleration3D  omega_dot = {
		1.0 * rad_per_second_squared,
		4.0 * rad_per_second_squared,
		9.0 * rad_per_second_squared};
	AngularVelocity3D      omega = {
		1.0 * rad_per_second,
		2.0 * rad_per_second,
		3.0 * rad_per_second};
	AngularVelocity3D result = omega_dot.cwiseSqrt();
	ASSERT_TRUE(util::is_approx(result, omega, 0.01));
}

TEST(Vector, SquaredNorm)
{
	Displacement3D d = {
		1.0 * meter,
		2.0 * meter,
		2.0 * meter};
	Area d_squared_norm = util::squared_norm(d);

	Area d_sn_expect = 9.0 * meter_squared;
	ASSERT_TRUE(util::is_approx(d_squared_norm, d_sn_expect, 0.01));
}

TEST(Vector, Norm)
{
	Displacement3D d = {
		1.0 * meter,
		2.0 * meter,
		2.0 * meter};
	Displacement d_norm = util::norm(d);

	Displacement d_expect = 3.0 * meter;
	ASSERT_TRUE(util::is_approx(d_norm, d_expect, 0.01));
}

TEST(Vector, Normalized)
{
	Displacement3D d = {
		1.0 * meter,
		2.0 * meter,
		2.0 * meter};
	Displacement3D d_normalized = util::normalized(d);

	Displacement3D d_expect = {
		1.0 / 3.0 * meter,
		2.0 / 3.0 * meter,
		2.0 / 3.0 * meter};
	ASSERT_TRUE(util::is_approx(d_normalized, d_expect, 0.01));
}

TEST(Vector, ScalarMultiplyVector)
{
	Displacement3D d = {
		1.0 * meter,
		2.0 * meter,
		2.0 * meter};
	Force s = 4.0 * newton;
	Torque3D result = s * d;

	Torque3D expect = {
		1.0 * 4.0 * newton_meter,
		2.0 * 4.0 * newton_meter,
		2.0 * 4.0 * newton_meter};
	ASSERT_TRUE(util::is_approx(result, expect, 0.01));
}

TEST(Vector, VectorMultiplyScalar)
{
	Displacement3D d = {
		1.0 * meter,
		2.0 * meter,
		2.0 * meter};
	Force s = 4.0 * newton;
	Torque3D result = d * s;

	Torque3D expect = {
		1.0 * 4.0 * newton_meter,
		2.0 * 4.0 * newton_meter,
		2.0 * 4.0 * newton_meter};
	ASSERT_TRUE(util::is_approx(result, expect, 0.01));
}

TEST(Vector, VectorDivideScalar)
{
	Torque3D d = {
		1.0 * 4.0 * newton_meter,
		2.0 * 4.0 * newton_meter,
		2.0 * 4.0 * newton_meter};

	Force s = 4.0 * newton;
	Displacement3D result = d / s;

	Displacement3D expect = {
		1.0 * meter,
		2.0 * meter,
		2.0 * meter};
	ASSERT_TRUE(util::is_approx(result, expect, 0.01));
}
