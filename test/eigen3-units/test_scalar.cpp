#include <gtest/gtest.h>
#include "matrix.h"

using namespace crobot::math;


TEST(Scalar, AddOperator)
{
	Displacement d1 = 1.0 * meter;
	Displacement d2 = 2.0 * meter;
	Displacement result = d1 + d2;
	Displacement expect = 3.0 * meter;

	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

TEST(Scalar, AddEqualOperator)
{
	Displacement result = 1.0 * meter;
	Displacement d2 = 2.0 * meter;
	result += d2;
	Displacement expect = 3.0 * meter;

	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

TEST(Scalar, SubOperator)
{
	Displacement d1 = 5.0 * meter;
	Displacement d2 = 2.0 * meter;
	Displacement result = d1 - d2;
	Displacement expect = 3.0 * meter;

	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

TEST(Scalar, SubEqualOperator)
{
	Displacement result = 5.0 * meter;
	Displacement d2 = 2.0 * meter;
	result -= d2;
	Displacement expect = 3.0 * meter;

	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

TEST(Scalar, MultiplyOperator)
{
	Displacement d1 = 3.0 * meter;
	Displacement d2 = 2.0 * meter;
	Area result = d1 * d2;
	Area expect = 6.0 * meter_squared;

	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

TEST(Scalar, DivideOperator)
{
	Area         a      = 2.0 * meter_squared;
	Displacement d      = 1.0 * meter;
	Displacement result = a / d;
	Displacement expect = 2.0 * meter;

	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

TEST(Scalar, OperatorMultiplyDoubleScalar)
{
	Displacement d      = 1.0 * meter;
	Displacement result = 2.0 * d;

	Displacement expect = 2.0 * meter;
	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

TEST(Scalar, OperatorMultiplyScalarDouble)
{
	Displacement d      = 1.0 * meter;
	Displacement result = d * 2.0;

	Displacement expect = 2.0 * meter;
	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

TEST(Scalar, OperatorMultiplyEqualScalarDouble)
{
	Displacement result = 1.0 * meter;
	result *= 2.0;

	Displacement expect = 2.0 * meter;
	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

TEST(Scalar, OperatorDivideScalarDouble)
{
	Displacement d      = 2.0 * meter;
	Displacement result = d / 2.0;

	Displacement expect = 1.0 * meter;
	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

TEST(Scalar, OperatorDivideEqualScalarDouble)
{
	Displacement result = 2.0 * meter;
	result /= 2.0;

	Displacement expect = 1.0 * meter;
	ASSERT_TRUE(util::is_approx(result, expect, 0.1));
}

