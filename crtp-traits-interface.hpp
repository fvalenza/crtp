 #ifndef _crtp_interface_hpp
 #define _crtp_interface_hpp

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
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };

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
    enum {
      LINEAR = traits<Derived>::LINEAR,
      ANGULAR = traits<Derived>::ANGULAR 
    };


  public:
    Derived(): rot(), trans() {};


    template<typename M3,typename v3>
    Derived(const Eigen::MatrixBase<M3> & R, const Eigen::MatrixBase<v3> & p) 
    : rot(R), trans(p)
    {
    }

    static Derived Random()
    {
      Eigen::Quaternion<_Scalar,_Options> q(Vector4::Random());
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

#endif