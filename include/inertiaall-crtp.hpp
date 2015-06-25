#ifndef __inertia_all_hpp__
#define __inertia_all_hpp__

#include "spatial-fwd.hpp"
#include "symmetric3.hpp"
namespace se3
{

    template< class Derived>
    class InertiaBase
    {
    protected:
    

      typedef Derived  Derived_t;
      typedef typename traits<Derived_t>::Scalar Scalar_t;
      typedef typename traits<Derived_t>::Vector3 Vector3;

    public:
      Derived_t & derived() { return *static_cast<Derived_t*>(this); }
      const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

      Scalar_t             mass()    const { return static_cast<const Derived_t*>(this)->mass(); }
      const Vector3 &    lever()   const { return static_cast<const Derived_t*>(this)->lever(); }
      const Symmetric3 & inertia() const { return static_cast<const Derived_t*>(this)->inertia(); }

    };


    // template <typename, int> class InertiaTpl; // class Derived exists.

    template<typename T, int U>
    struct traits< InertiaTpl<T, U> >
    {
      typedef T Scalar;
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
      typedef MotionTpl<T,U> Motion;
      enum {
        LINEAR = 0,
        ANGULAR = 3
      };
      // typedef typename Derived<T, U>::Vector3 Linear_t;
    };

    template<typename _Scalar, int _Options>
    class InertiaTpl : public InertiaBase< InertiaTpl< _Scalar, _Options > >
    {

      // These typename can be put in a Macro ( see joint-base)
      typedef typename traits<InertiaTpl>::Scalar Scalar;
      typedef typename traits<InertiaTpl>::Vector3 Vector3;
      typedef typename traits<InertiaTpl>::Vector4 Vector4;
      typedef typename traits<InertiaTpl>::Vector6 Vector6;
      typedef typename traits<InertiaTpl>::Matrix3 Matrix3;
      typedef typename traits<InertiaTpl>::Matrix4 Matrix4;
      typedef typename traits<InertiaTpl>::Matrix6 Matrix6;
      typedef typename traits<InertiaTpl>::ActionMatrix ActionMatrix_t;
      typedef typename traits<InertiaTpl>::Angular_t Angular_t;
      typedef typename traits<InertiaTpl>::Linear_t Linear_t;
      typedef typename traits<InertiaTpl>::Quaternion Quaternion_t;
      typedef typename traits<InertiaTpl>::SE3 SE3;
      typedef typename traits<InertiaTpl>::Force Force;
      typedef typename traits<InertiaTpl>::Motion Motion;
      enum {
        LINEAR = traits<InertiaTpl>::LINEAR,
        ANGULAR = traits<InertiaTpl>::ANGULAR 
      };


    public:
      
      // Constructors
      InertiaTpl() : m(), c(), I() {}

      InertiaTpl(Scalar m_, 
         const Vector3 &c_, 
         const Matrix3 &I_)
        : m(m_),
          c(c_),
          I(I_)
      {

      }

      InertiaTpl(Scalar _m, 
         const Vector3 &_c, 
         const Symmetric3 &_I)
        : m(_m),
          c(_c),
          I(_I)
      {

      }

      InertiaTpl(const InertiaTpl & clone)  // Clone constructor for std::vector 
      : m(clone.m),
        c(clone.c),
        I(clone.I)    
      {

      }

      InertiaTpl& operator= (const InertiaTpl& clone) // Copy operator for std::vector 
      {
        m=clone.m; c=clone.c; I=clone.I;
        return *this;
      }

      /* Requiered by std::vector boost::python bindings. */
      bool operator==( const InertiaTpl& Y2 ) 
      { 
        return (m==Y2.m) && (c==Y2.c) && (I==Y2.I);
      }

      template<typename S2,int O2>
      InertiaTpl( const InertiaTpl<S2,O2> & clone )
        : m(clone.mass()),
          c(clone.lever()),
          I(clone.inertia().matrix())
      {

      }

      // Initializers
      static InertiaTpl Zero() 
      {
        return InertiaTpl(0., 
                          Vector3::Zero(), 
                          Symmetric3::Zero());
      }

      static InertiaTpl Identity() 
      {
        return InertiaTpl(1., 
                          Vector3::Zero(), 
                          Symmetric3::Identity());
      }

      static InertiaTpl Random()
      {
        /* We have to shoot "I" definite positive and not only symmetric. */
        return InertiaTpl(Eigen::internal::random<Scalar>()+1,
                          Vector3::Random(),
                          Symmetric3::RandomPositive());
      }

      Matrix6 matrix() const
      {
        Matrix6 M;
        const Matrix3 & c_cross = (skew(c));
        M.template block<3,3>(LINEAR, LINEAR ).template setZero ();
        M.template block<3,3>(LINEAR, LINEAR ).template diagonal ().template fill (m);
        M.template block<3,3>(ANGULAR,LINEAR ) = m * c_cross;
        M.template block<3,3>(LINEAR, ANGULAR) = -M.template block<3,3> (ANGULAR, LINEAR);
        M.template block<3,3>(ANGULAR,ANGULAR) = (Matrix3)(I - M.template block<3,3>(ANGULAR, LINEAR) * c_cross);

        return M;
      }
      operator Matrix6 () const { return matrix(); }

      // Arithmetic operators
      friend InertiaTpl operator+(const InertiaTpl &Ya, const InertiaTpl &Yb)
      {
        /* Y_{a+b} = ( m_a+m_b,
         *             (m_a*c_a + m_b*c_b ) / (m_a + m_b),
         *             I_a + I_b - (m_a*m_b)/(m_a+m_b) * AB_x * AB_x )
         */

        const double & mab = Ya.m+Yb.m;
        const Vector3 & AB = (Ya.c-Yb.c).eval();
        return InertiaTpl( mab,
         (Ya.m*Ya.c+Yb.m*Yb.c)/mab,
         Ya.I+Yb.I - (Ya.m*Yb.m/mab)* typename Symmetric3::SkewSquare(AB));
      }

      InertiaTpl& operator+=(const InertiaTpl &Yb)
      {
        const InertiaTpl& Ya = *this;
        const double & mab = Ya.m+Yb.m;
        const Vector3 & AB = (Ya.c-Yb.c).eval();
        c *= m; c += Yb.m*Yb.c; c /= mab;
        I += Yb.I; I -= (Ya.m*Yb.m/mab)* typename Symmetric3::SkewSquare(AB);
        m  = mab;
        return *this;
      }

      Force operator*(const Motion &v) const 
      {
        const Vector3 & mcxw = m*c.cross(v.angular());
        return Force( m*v.linear()-mcxw,
                      m*c.cross(v.linear()) + I*v.angular() - c.cross(mcxw) );
      }

      /// aI = aXb.act(bI)
      InertiaTpl se3Action(const SE3 & M) const
      {
        /* The multiplication RIR' has a particular form that could be used, however it
         * does not seems to be more efficient, see http://stackoverflow.com/questions/
         * 13215467/eigen-best-way-to-evaluate-asa-transpose-and-store-the-result-in-a-symmetric .*/
        return InertiaTpl(m,
                          M.translation()+M.rotation()*c,
                          I.rotate(M.rotation()) );
      }

      /// bI = aXb.actInv(aI)
      InertiaTpl se3ActionInverse(const SE3 & M) const
      {
        return InertiaTpl(m,
                          M.rotation().transpose()*(c-M.translation()),
                          I.rotate(M.rotation().transpose()) );
      }

      Force vxiv( const Motion& v ) const 
      {
        const Vector3 & mcxw = m*c.cross(v.angular());
        const Vector3 & mv_mcxw = m*v.linear()-mcxw;
        return Force( v.angular().cross(mv_mcxw),
                      v.angular().cross(c.cross(mv_mcxw)+I*v.angular())-v.linear().cross(mcxw) );
      }

      friend std::ostream & operator<< (std::ostream &os, const InertiaTpl &I)
      {
        os  << "m =" << I.m << ";\n"
            << "c = [\n" << I.c.transpose() << "]';\n"
            << "I = [\n" << (Matrix3)I.I << "];";
        return os;
      }

      // Getters
      _Scalar             mass()    const { return m; }
      const Vector3 &    lever()   const { return c; }
      const Symmetric3 & inertia() const { return I; }


    protected:
      _Scalar m;
      Vector3 c;
      Symmetric3 I;
    };


typedef InertiaTpl<double,0> Inertiad;

}

#endif //ifndef __inertia_all_hpp__
