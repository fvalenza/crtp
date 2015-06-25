#ifndef _force_all_hpp
#define _force_all_hpp

#include "spatial-fwd.hpp"
namespace se3
{


   // template<class C> struct traits {};
  

    template< class Derived>
    class ForceBase
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


    // template <typename, int> class SE3; // class Derived exists.

    template<typename T, int U>
    struct traits< ForceTpl<T, U> >
    {
      typedef Eigen::Matrix<T,3,1,U> Vector3;
      typedef Eigen::Matrix<T,4,1,U> Vector4;
      typedef Eigen::Matrix<T,6,1,U> Vector6;
      typedef Eigen::Matrix<T,3,3,U> Matrix3;
      typedef Eigen::Matrix<T,4,4,U> Matrix4;
      typedef Eigen::Matrix<T,6,6,U> Matrix6;
      typedef Vector3 Angular_t;
      typedef Vector3 Linear_t;
      typedef Eigen::Quaternion<T,U> Quaternion;
      typedef SE3Tpl<T,U> SE3;
      enum {
        LINEAR = 0,
        ANGULAR = 3
      };
      // typedef typename Derived<T, U>::Vector3 Linear_t;
    };

    template<typename _Scalar, int _Options>
    class ForceTpl : public ForceBase< ForceTpl< _Scalar, _Options > >
    {

      // These typename can be put in a Macro ( see joint-base)
      typedef typename traits<ForceTpl>::Vector3 Vector3;
      typedef typename traits<ForceTpl>::Vector4 Vector4;
      typedef typename traits<ForceTpl>::Vector6 Vector6;
      typedef typename traits<ForceTpl>::Matrix3 Matrix3;
      typedef typename traits<ForceTpl>::Matrix4 Matrix4;
      typedef typename traits<ForceTpl>::Matrix6 Matrix6;
      typedef typename traits<ForceTpl>::Angular_t Angular_t;
      typedef typename traits<ForceTpl>::Linear_t Linear_t;
      typedef typename traits<ForceTpl>::Quaternion Quaternion_t;
      typedef typename traits<ForceTpl>::SE3 SE3;
      enum {
        LINEAR = traits<ForceTpl>::LINEAR,
        ANGULAR = traits<ForceTpl>::ANGULAR 
      };


    public:
      
      ForceTpl() : m_n(), m_f() {}

      template<typename f3_t,typename n3_t>
      ForceTpl(const Eigen::MatrixBase<f3_t> & f,const Eigen::MatrixBase<n3_t> & n)
        : m_n(n),
          m_f(f)
      {

      }

      template<typename f6>
      explicit ForceTpl(const Eigen::MatrixBase<f6> & f)
        : m_n(f.template segment<3>(ANGULAR)),
          m_f(f.template segment<3>(LINEAR)) 
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(f6);
        assert( f.size() == 6 );
      }

      //copy constructor
      ForceTpl(const ForceTpl & clone)
        : m_n(clone.angular()),
          m_f(clone.linear())
      {

      }

      static ForceTpl Zero() { return ForceTpl(Linear_t::Zero(), Angular_t::Zero()); }
      static ForceTpl Random() { return ForceTpl(Linear_t::Random(), Angular_t::Random()); }

      ForceTpl & setZero () { m_n.setZero (); m_f.setZero (); return *this; } 
      ForceTpl & setRandom () { m_n.setRandom (); m_f.setRandom (); return *this; }

      Vector6 toVector() const
      {
        Vector6 f;
        f.segment<3>(ANGULAR) = m_n;
        f.segment<3>(LINEAR)  = m_f;
        return f;
      }
      operator Vector6 () const { return toVector(); }

      ForceTpl & operator= (const ForceTpl & other)
      {
        m_n = other.angular();
        m_f = other.linear();
        return *this;
      }

      template<typename F6>
      ForceTpl & operator=(const Eigen::MatrixBase<F6> & phi)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(F6); assert(phi.size() == 6);
        m_n = phi.template segment<3>(ANGULAR);
        m_f = phi.template segment<3>(LINEAR);
        return *this;
      }

      ForceTpl operator+(const ForceTpl & phi) const
      {
        return ForceTpl(m_f+phi.m_f,m_n+phi.m_n);
      }

      ForceTpl& operator+= (const ForceTpl & phi)
      {
        m_f += phi.m_f;
        m_n += phi.m_n;
        return *this;
      }

      ForceTpl operator-() const
      {
        return ForceTpl(-m_f, -m_n);
      }

      ForceTpl operator-(const ForceTpl & phi) const
      {
        return ForceTpl(m_f-phi.m_f,m_n-phi.m_n);
      }

      //tyemplate <typename Scalar> .. operator*(Scalar a) ??
      ForceTpl operator*(double a) const
      {
        return ForceTpl(m_f*a, m_n*a);
      }

      template <typename D>
      ForceTpl operator + (ForceBase<D> a){
        return ForceTpl(m_f+a.linear(), m_n + a.angular());
      }
      // friend ForceTpl operator*(Scalar a, const ForceTpl & phi)
      // {
      //   return ForceTpl(phi.n()*a, phi.f()*a);
      // }

      /// af = aXb.act(bf)
      ForceTpl se3Action(const SE3 & m) const
      {
        Vector3 Rf (static_cast<Vector3>( (m.rotation()) * linear() ) );
        return ForceTpl(Rf,m.translation().cross(Rf)+m.rotation()*angular());
      }

      ForceTpl se3ActionInverse(const SE3 & m) const
      {
        return ForceTpl(m.rotation().transpose()*linear(),
            m.rotation().transpose()*(angular() - m.translation().cross(linear())) );
      }

      friend std::ostream & operator << (std::ostream & os, const ForceTpl & phi)
      {
        os
        << "f =\n" << phi.linear() << std::endl
        << "tau =\n" << phi.angular() << std::endl;
        return os;
      }


      const Angular_t & angular() const { return m_n; }
      Angular_t & angular() { return m_n; }
      void angular(const Angular_t & R) { m_n = R; }
      const Linear_t & linear() const { return m_f;}
      Linear_t & linear() { return m_f;}
      void linear(const Linear_t & p) { m_f=p; }

    protected:
      Angular_t m_n;
      Linear_t m_f;
    };



typedef ForceTpl<double,0> Forced;

}

#endif
