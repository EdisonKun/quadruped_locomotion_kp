#pragma once

#include "kindr/Core"
#include "common/typedefs.hpp"

struct EnumClassHash //use a function object to calculate hash of enum class
{
  template<typename T>
  std::size_t operator()(T t) const//Reload (). this struct will return the T,t static_cast.
  {
    /**
      /* enforce to transform.
       * static_cast <type_id>(expression)
       * transform expression to the type_id
      */
    return static_cast<std::size_t>(t);//<std::size_t> the maximum range of the system
  }
};

using namespace romo;//in the file of common/typedefs.hpp

namespace quadruped_model_kp {
typedef kindr::VectorTypeless<double,18> GeneralizedCoordinates;
typedef kindr::VectorTypeless<double,18> GeneralizedVelocities;
typedef kindr::VectorTypeless<double,18> GeneralizedAccelerations;
typedef kindr::VectorTypeless<double,3> JointPositionLimb;
typedef kindr::VectorTypeless<double,3> JointVelocitiesLimb;
typedef kindr::VectorTypeless<double,3> JointAccelationsLimb;
typedef kindr::VectorTypeless<double,3> JointTorquesLimb;
typedef kindr::VectorTypeless<double,12> JointPositions;
typedef kindr::VectorTypeless<double,12> JointVelocities;
typedef kindr::VectorTypeless<double,12> JointAccelations;
typedef kindr::VectorTypeless<double,12> JointTorques;

class QuadrupedModel_kp
{
public:
  QuadrupedModel_kp();
  ~QuadrupedModel_kp();

  class QuadrupedDescription_kp
  {
  private:
    /*data*/
  public:
    QuadrupedDescription_kp();
    virtual ~QuadrupedDescription_kp();
    enum class LimbEnum
    {
      LF_LEG,
      RF_LEG,
      RH_LEG,
      LH_LEG,
    };

    enum class BranchEnum
    {
      BASE,
      LF_LEG,
      RF_LEG,
      RH_LEG,
      LH_LEG,
    };

    enum class JointNodeEnum
    {
      LF_LEG_HAA,
      LF_LEG_HFE,
      LF_LEG_KFE,
      RF_LEG_HAA,
      RF_LEG_HFE,
      RF_LEG_KFE,
      RH_LEG_HAA,
      RH_LEG_HFE,
      RH_LEG_KFE,
      LH_LEG_HAA,
      LH_LEG_HFE,
      LH_LEG_KFE,
    };

    static BranchEnum mapEnum(LimbEnum le)
    {
      int index = static_cast<int>(le);
      index = index + 1;
      return static_cast<BranchEnum>(index);
    }

    static int getNumDofLimb()
    {
      return 3;
    }

    static int getLimbStartIndexInJ(LimbEnum limb)
    {
      switch(limb){
      case LimbEnum::LF_LEG:
        return 0;
      case LimbEnum::RF_LEG:
        return 3;
      case LimbEnum::RH_LEG:
        return 6;
      case LimbEnum::LH_LEG:
        return 9;
      }
    }
  };

private:
  /*data*/
};

}
