//
// Copyright (c) 2018, Yuechuan Xue
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef CROBOT_MATRIX_H
#define CROBOT_MATRIX_H
#include <boost/preprocessor/stringize.hpp>
#include <Eigen/Core>
// for cross product
#include <Eigen/Geometry>
#include <boost/units/quantity.hpp>
#include <boost/units/limits.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/time.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/si/acceleration.hpp>
#include <boost/units/systems/si/force.hpp>
#include <boost/units/systems/si/frequency.hpp>
#include <boost/units/systems/si/energy.hpp>
#include <boost/units/systems/si/area.hpp>
#include <boost/units/systems/si/io.hpp>
#include <boost/units/systems/si/dimensionless.hpp>
#include <complex.h>

namespace Eigen 
{
	using boost::units::quantity;
	using boost::units::multiply_typeof_helper;

	template< typename T1, typename T2 >
	struct ScalarBinaryOpTraits<quantity<T1>, T2,
		internal::scalar_product_op<quantity<T1>, T2 > >
	{
		typedef typename multiply_typeof_helper<T1, T2>::type ReturnType;
	};

	template< typename T1, typename T2 >
	struct ScalarBinaryOpTraits< T1, quantity<T2>,
		internal::scalar_product_op< T1, quantity<T2> >>
	{
		typedef typename multiply_typeof_helper<T1, T2>::type ReturnType;
	};

	template< typename T1, typename T2 >
	struct ScalarBinaryOpTraits< quantity<T1>, quantity<T2>,
		internal::scalar_product_op<quantity<T1>, quantity<T2>> >
	{
		typedef quantity<typename multiply_typeof_helper<T1, T2>::type> ReturnType;
	};

	template< typename T1, typename T2 >
	struct ScalarBinaryOpTraits<quantity<T1>, T2,
		internal::scalar_conj_product_op<quantity<T1>, T2 >>
	{
		typedef typename multiply_typeof_helper<T1, T2>::type ReturnType;
	};

	template< typename T1, typename T2 >
	struct ScalarBinaryOpTraits< T1, quantity<T2>,
		internal::scalar_conj_product_op< T1, quantity<T2> >>
	{
		typedef typename multiply_typeof_helper<T1, T2>::type ReturnType;
	};

	template< typename T1, typename T2 >
	struct ScalarBinaryOpTraits< quantity<T1>, quantity<T2>,
		internal::scalar_conj_product_op< quantity<T1>, quantity<T2> >>
	{
		typedef quantity<typename multiply_typeof_helper<T1, T2>::type> ReturnType;
	};

	using boost::units::divide_typeof_helper;
	template< typename T1, typename T2 >
	struct ScalarBinaryOpTraits< quantity<T1>, T2,
		internal::scalar_quotient_op< quantity<T1>, T2> > 
	{
		typedef typename divide_typeof_helper<T1, T2>::type ReturnType;
	};
	template< typename T1, typename T2 >
	struct ScalarBinaryOpTraits< T1, quantity<T2>,
		internal::scalar_quotient_op< T1, quantity<T2> >> 
	{
		typedef typename divide_typeof_helper<T1, T2>::type ReturnType;
	};
	template< typename T1, typename T2 >
	struct ScalarBinaryOpTraits< quantity<T1>, quantity<T2>,
		internal::scalar_quotient_op< quantity<T1>,	quantity<T2> >> 
	{
		typedef quantity<typename divide_typeof_helper<T1, T2>::type> ReturnType;
	};

	template<typename T>
	struct NumTraits<quantity<T>> : NumTraits < double >
	{
		typedef quantity<T> Real;
		typedef quantity<T> NonInteger;
		typedef quantity<T> Nested;

		enum {
			IsComplex             = 0,
			IsInteger             = 0,
			IsSigned              = 1,
			RequireInitialization = 1,
			ReadCost              = 1,
			AddCost               = 3,
			MulCost               = 3
		};
	};

	// sqrt
	using boost::units::root_typeof_helper;
	using boost::units::static_rational;
	namespace numext {
		template<typename Unit, typename Scalar>
		EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE
		typename root_typeof_helper< quantity<Unit, Scalar>, static_rational<2>>::type
		sqrt(const quantity<Unit, Scalar> &x)
		{
			using boost::units::sqrt;
			return sqrt(x);
		}
	} // namespace numext

	namespace internal
	{
		template<typename Unit, typename Scalar>
		struct scalar_sqrt_op<quantity<Unit, Scalar>>
		{
			EIGEN_EMPTY_STRUCT_CTOR(scalar_sqrt_op)
			using result_type =
			typename root_typeof_helper< quantity<Unit, Scalar>, static_rational<2>>::type;

			EIGEN_DEVICE_FUNC 
			result_type operator() (const quantity<Unit, Scalar>& a) const
			{
				return numext::sqrt(a);
			}

			template <typename Packet>
			EIGEN_DEVICE_FUNC
			Packet packetOp(const Packet& a) const
			{
				return internal::psqrt(a);
			}
		};
	} // namespace internal

	template< typename Unit, typename Scalar >
	typename multiply_typeof_helper< quantity<Unit, Scalar>, quantity<Unit, Scalar> >::type
	abs2_ext(const quantity<Unit, Scalar> & x)
	{
		return x * x;
	}

	namespace numext
	{
		template<typename Unit, typename Scalar >
		typename multiply_typeof_helper<quantity<Unit, Scalar>,	quantity<Unit, Scalar> >::type
		abs2(const quantity<Unit, Scalar> & x)
		{
			return abs2_ext(x);
		}
	} //namespace numext

	namespace internal
	{
		template<typename Unit, typename Scalar>
		struct abs2_impl<quantity<Unit, Scalar>>
		{
			EIGEN_DEVICE_FUNC
			static typename multiply_typeof_helper<
			quantity<Unit, Scalar>, quantity<Unit, Scalar>>::type
			run(const quantity<Unit, Scalar>& x) 
			{
				return x * x;
			}
		};

		template<typename Unit, typename Scalar>
		struct scalar_abs2_op<quantity<Unit, Scalar>>
		{
			EIGEN_EMPTY_STRUCT_CTOR(scalar_abs2_op)
			using result_type = typename multiply_typeof_helper<
			quantity<Unit, Scalar>,	quantity<Unit, Scalar>>::type;

			EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
			result_type operator() (const quantity<Unit, Scalar>& a) const
			{
				return numext::abs2(a);
			}

			template<typename Packet>
			EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
			Packet packetOp(const Packet& a) const
			{
				return internal::pmul(a, a);
			}
		};
	} // namespace internal
} // namespace Eigen

namespace crobot
{
	namespace math
	{
		struct util
		{
			template<typename T, int Row, int Col>
			EIGEN_STRONG_INLINE
			static typename boost::units::multiply_typeof_helper<T, T>::type
			squared_norm(const Eigen::Matrix<T, Row, Col>& m)
			{
				return Eigen::numext::real(m.cwiseAbs2().sum());
			}

			template<typename T, int Row, int Col>
			EIGEN_STRONG_INLINE
			static T
			norm(const Eigen::Matrix<T, Row, Col>& m)
			{
				return Eigen::numext::sqrt(squared_norm(m));
			}

			template<typename T, int Row, int Col>
			EIGEN_STRONG_INLINE
			static typename Eigen::Matrix<T, Row, Col>
			normalized(const Eigen::Matrix<T, Row, Col>& m)
			{
				using SquaredType =
					typename boost::units::multiply_typeof_helper<T, T>::type;
				SquaredType z = squared_norm(m);
				if (z.value() > 0.0)
				{
					return m / Eigen::numext::sqrt(z).value();
				}
				return m;
			}
		};

#define CROBOT_UNITS_STATIC_CONSTANT(name, type)                       \
	template<bool b>                                                   \
	struct name##_instance_t                                           \
	{                                                                  \
		static const type instance;                                    \
	};                                                                 \
																       \
	namespace                                                          \
	{                                                                   \
		static const type& name = name##_instance_t<true>::instance;    \
	}                                                                   \
																        \
	template<bool b>                                                    \
	const type name##_instance_t<b>::instance
// CROBOT_UNITS_STATIC_CONSTANT

#define CROBOT_BOOST_UNITS_REDEFINE( NAME , TYPE , UNIT )               \
	using NAME##1I = boost::units::quantity< TYPE , int>;               \
	using NAME##1F = boost::units::quantity< TYPE , float>;             \
	using NAME##1D = boost::units::quantity< TYPE , double>;            \
	using NAME     = NAME##1D;                                          \
	CROBOT_UNITS_STATIC_CONSTANT( UNIT , TYPE )
// CROBOT_BOOST_UNITS_REDEFINE

		CROBOT_BOOST_UNITS_REDEFINE(Time, boost::units::si::time, second);
		CROBOT_BOOST_UNITS_REDEFINE(Area, boost::units::si::area, meter_squared);
		CROBOT_BOOST_UNITS_REDEFINE(Frequency, boost::units::si::frequency, hertz);

#define CROBOT_PHYSICS_VECTOR_DEFINE( NAME, TYPE , UNIT )              \
		template<typename T>                                           \
		using NAME##Type = boost::units::quantity< TYPE , T>;          \
    	template<typename T, int Dim>                                  \
        using NAME##_ = Eigen::Matrix<NAME##Type<T>, Dim, 1>;          \
        template<typename T>                                           \
        using NAME##2_  = NAME##_ <T, 2>;                              \
        template<typename T>                                           \
        using NAME##3_ = NAME##_ <T, 3>;                               \
        using NAME##2I = NAME##2_<int>;                                \
        using NAME##2F = NAME##2_<float>;                              \
        using NAME##2D = NAME##2_<double>;                             \
		using NAME##2  = NAME##2D;                                     \
        using NAME##3I = NAME##3_<int>;                                \
        using NAME##3F = NAME##3_<float>;                              \
        using NAME##3D = NAME##3_<double>;                             \
		using NAME##3  = NAME##3D;                                     \
		CROBOT_BOOST_UNITS_REDEFINE( NAME , TYPE , UNIT )
// CROBOT_PHYSICS_VECTOR_DEFINE( NAME )
        
		CROBOT_PHYSICS_VECTOR_DEFINE(Force,        boost::units::si::force,        newton);
		CROBOT_PHYSICS_VECTOR_DEFINE(Torque,       boost::units::si::energy,       joule);
		CROBOT_PHYSICS_VECTOR_DEFINE(Displacement, boost::units::si::length,       meter);
		CROBOT_PHYSICS_VECTOR_DEFINE(Velocity,     boost::units::si::velocity,     meter_per_second);
		CROBOT_PHYSICS_VECTOR_DEFINE(Acceleration, boost::units::si::acceleration, meter_per_second_squared);
		using angular_velocity_dimension =
		boost::units::derived_dimension<boost::units::time_base_dimension, -1>::type;
		using angular_velocity = 
		boost::units::unit<angular_velocity_dimension, boost::units::si::system>;
		CROBOT_PHYSICS_VECTOR_DEFINE(AngularVelocity, angular_velocity, rad_per_second);
		using angular_acceleration_dimension =
		boost::units::derived_dimension<boost::units::time_base_dimension, -2>::type;
		using angular_acceleration = 
		boost::units::unit<angular_acceleration_dimension, boost::units::si::system>;
		CROBOT_PHYSICS_VECTOR_DEFINE(AngularAcceleration, angular_acceleration, rad_per_second_squared);
		using angle =
		boost::units::unit<boost::units::dimensionless_type, boost::units::si::system>;
		CROBOT_PHYSICS_VECTOR_DEFINE(Angle, angle, rad);
	}
}

#endif /* CROBOT_MATRIX_H */
