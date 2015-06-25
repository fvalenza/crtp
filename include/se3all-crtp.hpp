#ifndef _se3_all_hpp
#define _se3_all_hpp

#include "spatial-fwd.hpp"
namespace se3
{

  /* Type returned by the "se3Action" and "se3ActionInverse" functions. */
  namespace internal 
  {
    template<typename D>
    struct ActionReturn    { typedef D Type; };
  }

  /** The rigid transform aMb can be seen in two ways: 
   *
   * - given a point p expressed in frame B by its coordinate vector Bp, aMb
   * computes its coordinates in frame A by Ap = aMb Bp.
   * - aMb displaces a solid S centered at frame A into the solid centered in
   * B. In particular, the origin of A is displaced at the origin of B: $^aM_b
   * ^aA = ^aB$.

   * The rigid displacement is stored as a rotation matrix and translation vector by:
   * aMb (x) =  aRb*x + aAB
   * where aAB is the vector from origin A to origin B expressed in coordinates A.
   */

   template<class C> struct traits {};
  

    template< class Derived>
    class SE3Base
    {
    protected:
    

      typedef Derived  Derived_t;
      typedef typename traits<Derived_t>::Angular_t Angular_t;
      typedef typename traits<Derived_t>::Linear_t Linear_t;

    public:
      Derived_t & derived() { return *static_cast<Derived_t*>(this); }
      const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

      const Angular_t & rotation() const  { return static_cast<const Derived_t*>(this)->rotation(); }
      const Linear_t & translation() const  { return static_cast<const Derived_t*>(this)->translation(); }
      Angular_t & rotation()  { return static_cast<Derived_t*>(this)->rotation(); }
      Linear_t & translation()   { return static_cast<Derived_t*>(this)->translation(); }
      void rotation(const Angular_t & R) { static_cast< Derived_t*>(this)->rotation(R); }
      void translation(const Linear_t & R) { static_cast< Derived_t*>(this)->translation(R); }

    };


    // template <typename, int> class SE3; // class Derived exists.

    template<typename T, int U>
    struct traits< SE3Tpl<T, U> >
    {
      typedef Eigen::Matrix<T,3,1,U> Vector3;
      typedef Eigen::Matrix<T,4,1,U> Vector4;
      typedef Eigen::Matrix<T,6,1,U> Vector6;
      typedef Eigen::Matrix<T,3,3,U> Matrix3;
      typedef Eigen::Matrix<T,4,4,U> Matrix4;
      typedef Eigen::Matrix<T,6,6,U> Matrix6;
      typedef Matrix3 Angular_t;
      typedef Vector3 Linear_t;
      typedef Eigen::Quaternion<T,U> Quaternion;
      enum {
        LINEAR = 0,
        ANGULAR = 3
      };
      // typedef typename Derived<T, U>::Vector3 Linear_t;
    };

    template<typename _Scalar, int _Options>
    class SE3Tpl : public SE3Base< SE3Tpl< _Scalar, _Options > >
    {

      // These typename can be put in a Macro ( see joint-base)
      typedef typename traits<SE3Tpl>::Vector3 Vector3;
      typedef typename traits<SE3Tpl>::Vector4 Vector4;
      typedef typename traits<SE3Tpl>::Vector6 Vector6;
      typedef typename traits<SE3Tpl>::Matrix3 Matrix3;
      typedef typename traits<SE3Tpl>::Matrix4 Matrix4;
      typedef typename traits<SE3Tpl>::Matrix6 Matrix6;
      typedef typename traits<SE3Tpl>::Angular_t Angular_t;
      typedef typename traits<SE3Tpl>::Linear_t Linear_t;
      typedef typename traits<SE3Tpl>::Quaternion Quaternion_t;
      enum {
        LINEAR = traits<SE3Tpl>::LINEAR,
        ANGULAR = traits<SE3Tpl>::ANGULAR 
      };


    public:
      SE3Tpl(): rot(), trans() {};


      template<typename M3,typename v3>
      SE3Tpl(const Eigen::MatrixBase<M3> & R, const Eigen::MatrixBase<v3> & p) 
      : rot(R), trans(p)
      {
      }

      SE3Tpl(int) : rot(Matrix3::Identity()), trans(Vector3::Zero()) {}

      SE3Tpl( const SE3Tpl & clone )  //cf SE3Tpl
        : rot(clone.rotation()),trans(clone.translation()) {}

      SE3Tpl & operator= (const SE3Tpl & other) // cf SE3TplTpl
      {
        rot = other.rotation ();
        trans = other.translation ();
        return *this;
      }

      static SE3Tpl Identity()
      {
        return SE3Tpl(1);
      }

      SE3Tpl & setIdentity () { rot.setIdentity (); trans.setZero (); return *this;}

      /// aXb = bXa.inverse()
      SE3Tpl inverse() const
      {
        return SE3Tpl(rot.transpose(), -rot.transpose()*trans);
      }

      static SE3Tpl Random()
      {
        Quaternion_t q(Vector4::Random());
        q.normalize();
        return SE3Tpl(q.matrix(),Vector3::Random());
      }

      SE3Tpl & setRandom ()
      {
        Quaternion_t q(Vector4::Random());
        q.normalize ();
        rot = q.matrix ();
        trans.setRandom ();

        return *this;
      }

      //  Unable to change this with Matrix4 toHomogen...
      Eigen::Matrix<double,4,4,0> toHomogeneousMatrix() const
      {
        Eigen::Matrix<double,4,4,0> M;
        M.block<3,3>(0,0) = rot;
        M.block<3,1>(0,3) = trans;
        M.block<1,3>(3,0).setZero();
        M(3,3) = 1;
        return M;
      }

      /// Vb.toVector() = bXa.toMatrix() * Va.toVector()
      Matrix6 toActionMatrix() const
      {
        Matrix6 M;
        M.block<3,3>(ANGULAR,ANGULAR)
        = M.block<3,3>(LINEAR,LINEAR) = rot;
        M.block<3,3>(ANGULAR,LINEAR).setZero();
        M.block<3,3>(LINEAR,ANGULAR)
        = skew(trans) * M.block<3,3>(ANGULAR,ANGULAR);
        return M;
      }

      void disp(std::ostream & os) const
      {
        os << "  R =\n" << rot << std::endl
        << "  p = " << trans.transpose() << std::endl;
      }

      /// --- GROUP ACTIONS ON M6, F6 and I6 --- 

     /// ay = aXb.act(by)
      template<typename D> typename internal::ActionReturn<D>::Type act   (const D & d) const 
      { 
        return d.se3Action(*this);
      }
        /// by = aXb.actInv(ay)
      template<typename D> typename internal::ActionReturn<D>::Type actInv(const D & d) const
      {
        return d.se3ActionInverse(*this);
      }

      Vector3 act   (const Vector3& p) const { return (rot*p+trans).eval(); }
      Vector3 actInv(const Vector3& p) const { return (rot.transpose()*(p-trans)).eval(); }

      SE3Tpl act    (const SE3Tpl& m2) const { return SE3Tpl( rot*m2.rot,trans+rot*m2.trans);}
      SE3Tpl actInv (const SE3Tpl& m2) const { return SE3Tpl( rot.transpose()*m2.rot, rot.transpose()*(m2.trans-trans));}

      /// Operators 
      operator Matrix4() const { return toHomogeneousMatrix(); }
      operator Matrix6() const { return toActionMatrix(); }

      friend std::ostream & operator << (std::ostream & os,const SE3Tpl & X)
      { X.disp(os); return os; }

      SE3Tpl operator*(const SE3Tpl & m2) const    { return this->act(m2); }



      const Angular_t & rotation() const { return rot; }
      Angular_t & rotation() { return rot; }
      void rotation(const Angular_t & R) { rot = R; }
      const Linear_t & translation() const { return trans;}
      Linear_t & translation() { return trans;}
      void translation(const Linear_t & p) { trans=p; }

    protected:
      Angular_t rot;
      Linear_t trans;
    };



typedef SE3Tpl<double,0> SE3d;

}

#endif
