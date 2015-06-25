#ifndef _se3_all_hpp
#define _se3_all_hpp

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
    class Base
    {
    protected:
    

      typedef Derived  Derived_t;
      typedef typename traits<Derived_t>::Angular_t Angular_t;
      typedef typename traits<Derived_t>::Linear_t Linear_t;

      Derived_t & derived() { return *static_cast<Derived_t*>(this); }
      const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

      const Angular_t & rotation() const  { return static_cast<const Derived_t*>(this)->rotation(); }
      const Linear_t & translation() const  { return static_cast<const Derived_t*>(this)->translation(); }
      Angular_t & rotation()  { return static_cast<Derived_t*>(this)->rotation(); }
      Linear_t & translation()   { return static_cast<Derived_t*>(this)->translation(); }
      void rotation(const Angular_t & R) { static_cast< Derived_t*>(this)->rotation(R); }
      void translation(const Linear_t & R) { static_cast< Derived_t*>(this)->translation(R); }

    };


    template <typename, int> class Derived; // class Derived exists.

    template<typename T, int U>
    struct traits< Derived<T, U> >
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
      // typedef typename Derived<T, U>::Vector3 Linear_t;
    };

    template<typename _Scalar, int _Options>
    class Derived : public Base< Derived< _Scalar, _Options > >
    {

      // These typename can be put in a Macro ( see joint-base)
      typedef typename traits<Derived>::Vector3 Vector3;
      typedef typename traits<Derived>::Vector4 Vector4;
      typedef typename traits<Derived>::Vector6 Vector6;
      typedef typename traits<Derived>::Matrix3 Matrix3;
      typedef typename traits<Derived>::Matrix4 Matrix4;
      typedef typename traits<Derived>::Matrix6 Matrix6;
      typedef typename traits<Derived>::Angular_t Angular_t;
      typedef typename traits<Derived>::Linear_t Linear_t;
      typedef typename traits<Derived>::Quaternion Quaternion_t;

    public:
      Derived(): rot(), trans() {};


      template<typename M3,typename v3>
      Derived(const Eigen::MatrixBase<M3> & R, const Eigen::MatrixBase<v3> & p) 
      : rot(R), trans(p)
      {
      }

      static Derived Random()
      {
        Quaternion_t q(Vector4::Random());
        q.normalize();
        return Derived(q.matrix(),Vector3::Random());
      }



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



typedef Derived<double,0> new_type;

}

#endif
