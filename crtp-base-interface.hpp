
  template< template <typename , int> class Derived, typename _Scalar, int _Options>
  class Base
  {

    typedef Eigen::Matrix<_Scalar,3,3,_Options> Matrix3;
    typedef Eigen::Matrix<_Scalar,4,4,_Options> Matrix4;
    typedef Eigen::Matrix<_Scalar,6,6,_Options> Matrix6;
    typedef Eigen::Matrix<_Scalar,3,1,_Options> Vector3;
    typedef Eigen::Matrix<_Scalar,4,1,_Options> Vector4;
    typedef Eigen::Matrix<_Scalar,6,1,_Options> Vector6;

   

    typedef Derived <_Scalar, _Options> Derived_t;
    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;

    // typedef typename traits<Derived_t>::type_t type_t;
    // typedef typename traits<Derived_t>::Angular_t Angular_t;
    // typedef typename traits<Derived_t>::Linear_t Linear_t;

    Derived_t & derived() { return *static_cast<Derived_t*>(this); }
    const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

    const Angular_t & rotation() const  { return static_cast<const Derived_t*>(this)->rotation(); }
    const Linear_t & translation() const  { return static_cast<const Derived_t*>(this)->translation(); }
    Angular_t & rotation()  { return static_cast<Derived_t*>(this)->rotation(); }
    Linear_t & translation()   { return static_cast<Derived_t*>(this)->translation(); }
    void rotation(const Angular_t & R) { static_cast< Derived_t*>(this)->rotation(R); }
    void translation(const Linear_t & R) { static_cast< Derived_t*>(this)->translation(R); }

  };


  // template <typename, int> class Derived; // class Derived exists.

  // template<typename T, int U>
  // struct traits< Derived<T, U> >
  // {
  //   typedef typename Derived<T, U>::Matrix3 Angular_t;
  //   typedef typename Derived<T, U>::Vector3 Linear_t;
  // };

  template<typename _Scalar, int _Options>
  class Derived : public Base<Derived, _Scalar, _Options>
  {

    typedef Eigen::Matrix<_Scalar,3,3,_Options> Matrix3;
    typedef Eigen::Matrix<_Scalar,4,4,_Options> Matrix4;
    typedef Eigen::Matrix<_Scalar,6,6,_Options> Matrix6;
    typedef Eigen::Matrix<_Scalar,3,1,_Options> Vector3;
    typedef Eigen::Matrix<_Scalar,4,1,_Options> Vector4;
    typedef Eigen::Matrix<_Scalar,6,1,_Options> Vector6;

    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;

    // typedef typename traits<Derived>::Angular_t Angular_t;
    // typedef typename traits<Derived>::Linear_t Linear_t;

    
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
    const Linear_t & translation() const { return trans;}
    Angular_t & rotation() { return rot; }
    Linear_t & translation() { return trans;}
    void rotation(const Angular_t & R) { rot = R; }
    void translation(const Linear_t & p) { trans=p; }


  protected:
    Angular_t rot;
    Linear_t trans;
  };



typedef Derived<double,0> new_type;
