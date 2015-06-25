#ifndef __motion_all_hpp__
#define __motion_all_hpp__

#include "spatial-fwd.hpp"
namespace se3
{

    template< class Derived>
    class MotionBase
    {
    protected:
    

      typedef Derived  Derived_t;
      typedef typename traits<Derived_t>::Angular_t Angular_t;
      typedef typename traits<Derived_t>::Linear_t Linear_t;

    public:
      Derived_t & derived() { return *static_cast<Derived_t*>(this); }
      const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

      const Angular_t & angular() const  { return static_cast<const Derived_t*>(this)->angular(); }
      const Linear_t & linear() const  { return static_cast<const Derived_t*>(this)->linear(); }
      Angular_t & angular()  { return static_cast<Derived_t*>(this)->angular(); }
      Linear_t & linear()   { return static_cast<Derived_t*>(this)->linear(); }
      void angular(const Angular_t & R) { static_cast< Derived_t*>(this)->angular(R); }
      void linear(const Linear_t & R) { static_cast< Derived_t*>(this)->linear(R); }

    };


    // template <typename, int> class MotionTpl; // class Derived exists.

    template<typename T, int U>
    struct traits< MotionTpl<T, U> >
    {
      typedef Eigen::Matrix<T,3,1,U> Vector3;
      typedef Eigen::Matrix<T,4,1,U> Vector4;
      typedef Eigen::Matrix<T,6,1,U> Vector6;
      typedef Eigen::Matrix<T,3,3,U> Matrix3;
      typedef Eigen::Matrix<T,4,4,U> Matrix4;
      typedef Eigen::Matrix<T,6,6,U> Matrix6;
      typedef Matrix6 ActionMatrix;
      typedef Vector3 Angular_t;
      typedef Vector3 Linear_t;
      typedef Eigen::Quaternion<T,U> Quaternion;
      typedef SE3Tpl<T,U> SE3;
      typedef ForceTpl<T,U> Force;
      enum {
        LINEAR = 0,
        ANGULAR = 3
      };
      // typedef typename Derived<T, U>::Vector3 Linear_t;
    };

    template<typename _Scalar, int _Options>
    class MotionTpl : public MotionBase< MotionTpl< _Scalar, _Options > >
    {

      // These typename can be put in a Macro ( see joint-base)
      typedef typename traits<MotionTpl>::Vector3 Vector3;
      typedef typename traits<MotionTpl>::Vector4 Vector4;
      typedef typename traits<MotionTpl>::Vector6 Vector6;
      typedef typename traits<MotionTpl>::Matrix3 Matrix3;
      typedef typename traits<MotionTpl>::Matrix4 Matrix4;
      typedef typename traits<MotionTpl>::Matrix6 Matrix6;
      typedef typename traits<MotionTpl>::ActionMatrix ActionMatrix_t;
      typedef typename traits<MotionTpl>::Angular_t Angular_t;
      typedef typename traits<MotionTpl>::Linear_t Linear_t;
      typedef typename traits<MotionTpl>::Quaternion Quaternion_t;
      typedef typename traits<MotionTpl>::SE3 SE3;
      typedef typename traits<MotionTpl>::Force Force;
      enum {
        LINEAR = traits<MotionTpl>::LINEAR,
        ANGULAR = traits<MotionTpl>::ANGULAR 
      };


    public:
      
      // Constructors
      MotionTpl() : m_w(), m_v() {}

      template<typename v1,typename v2> // MotionTpl(Linear_t &, Angular_t &) ??
      MotionTpl(const Eigen::MatrixBase<v1> & v, const Eigen::MatrixBase<v2> & w)
        : m_w(w), m_v(v) {}

      template<typename v6>
      explicit MotionTpl(const Eigen::MatrixBase<v6> & v)
        : m_w(v.template segment<3>(ANGULAR))
        , m_v(v.template segment<3>(LINEAR)) 
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(v6);
        assert( v.size() == 6 );
      }

      template<typename S2,int O2>
      explicit MotionTpl(const MotionTpl<S2,O2> & clone)
        : m_w(clone.angular()),m_v(clone.linear()) {}

      // initializers
      static MotionTpl Zero()   { return MotionTpl(Linear_t::Zero(),  Angular_t::Zero());   }
      static MotionTpl Random() { return MotionTpl(Linear_t::Random(),Angular_t::Random()); }

      MotionTpl & setZero () { m_v.setZero (); m_w.setZero (); return *this; }
      MotionTpl & setRandom () { m_v.setRandom (); m_w.setRandom (); return *this; }

      Vector6 toVector() const
      {
        Vector6 v;
        v.template segment<3>(ANGULAR) = m_w;
        v.template segment<3>(LINEAR)  = m_v;
        return v;
      }
      operator Vector6 () const { return toVector(); }

      ActionMatrix_t toActionMatrix () const
      {
        ActionMatrix_t X;
        X.block <3,3> (ANGULAR, ANGULAR) = X.block <3,3> (LINEAR, LINEAR) = skew (m_w);
        X.block <3,3> (LINEAR, ANGULAR) = skew (m_v);
        X.block <3,3> (ANGULAR, LINEAR).setZero ();

        return X;
      }

      // Arithmetic operators
      template<typename S2, int O2>
      MotionTpl & operator= (const MotionTpl<S2,O2> & other)
      {
        m_w = other.angular ();
        m_v = other.linear ();
        return *this;
      }
      
      template<typename V6>
      MotionTpl & operator=(const Eigen::MatrixBase<V6> & v)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(V6); assert(v.size() == 6);
        m_w = v.template segment<3>(ANGULAR);
        m_v = v.template segment<3>(LINEAR);
        return *this;
      }

      MotionTpl operator-() const
      {
        return MotionTpl(-m_v, -m_w);
      }

      MotionTpl operator+(const MotionTpl & v2) const
      {
        return MotionTpl(m_v+v2.m_v,m_w+v2.m_w);
      }
      MotionTpl operator-(const MotionTpl & v2) const
      {
        return MotionTpl(m_v-v2.m_v,m_w-v2.m_w);
      }
      MotionTpl& operator+=(const MotionTpl & v2)
      {
        m_v+=v2.m_v;
        m_w+=v2.m_w;
        return *this;
      }

      MotionTpl cross(const MotionTpl& v2) const
      {
        return MotionTpl( m_v.cross(v2.m_w)+m_w.cross(v2.m_v),
        m_w.cross(v2.m_w) );
      }

      // ForceTpl<_Scalar, _Options> cross(const ForceTpl<_Scalar, _Options>& phi )
      Force cross(const Force& phi) const
      {
        return Force( m_w.cross(phi.linear()),
                      m_w.cross(phi.angular())+m_v.cross(phi.linear()) );
        // return ForceTpl<Scalar,Options>
        //       ( m_w.cross(phi.linear()),
        //         m_w.cross(phi.angular())+m_v.cross(phi.linear()) );
      }


      // MotionTpl operator*(Scalar a) const
      // {
      //   return MotionTpl(m_w*a, m_v*a);
      // }

      // friend MotionTpl operator*(Scalar a, const MotionTpl & mv)
      // {
      //   return MotionTpl(mv.w()*a, mv.v()*a);
      // }

      MotionTpl se3Action(const SE3 & m) const
      {
        Vector3 Rw (static_cast<Vector3>(m.rotation() * angular()));
        return MotionTpl(m.rotation()*linear() + m.translation().cross(Rw), Rw);
      }
      /// bv = aXb.actInv(av)
      MotionTpl se3ActionInverse(const SE3 & m) const
      {
        return MotionTpl(m.rotation().transpose()*(linear()-m.translation().cross(angular())),
             m.rotation().transpose()*angular());
      }

      friend std::ostream & operator << (std::ostream & os, const MotionTpl & mv)
      {
        os << "  v = " << mv.linear().transpose () << std::endl
        << "  w = " << mv.angular().transpose () << std::endl;
        return os;
      }

      /** \brief Compute the classical acceleration of point according to the spatial velocity and spatial acceleration of the frame centered on this point
     */
      static inline Vector3 computeLinearClassicalAcceleration (const MotionTpl & spatial_velocity, const MotionTpl & spatial_acceleration)
      {
        return spatial_acceleration.linear () + spatial_velocity.angular ().cross (spatial_velocity.linear ());
      }

      /**
        \brief Compute the spatial motion quantity of the parallel frame translated by translation_vector */
      MotionTpl translate (const Vector3 & translation_vector) const
      {
        return MotionTpl (m_v + m_w.cross (translation_vector), m_w);
      }




      const Angular_t & angular() const { return m_w; }
      Angular_t & angular() { return m_w; }
      void angular(const Angular_t & R) { m_w = R; }
      const Linear_t & linear() const { return m_v;}
      Linear_t & linear() { return m_v;}
      void linear(const Linear_t & p) { m_v=p; }

    protected:
      Angular_t m_w;
      Linear_t m_v;
    };

    template<typename S,int O>
    MotionTpl<S,O> operator^( const MotionTpl<S,O> &m1, const MotionTpl<S,O> &m2 ) { return m1.cross(m2); }
    template<typename S,int O>
    ForceTpl<S,O> operator^( const MotionTpl<S,O> &m, const ForceTpl<S,O> &f ) { return m.cross(f); }


typedef MotionTpl<double,0> Motiond;

}

#endif //ifndef __motion_all_hpp__
