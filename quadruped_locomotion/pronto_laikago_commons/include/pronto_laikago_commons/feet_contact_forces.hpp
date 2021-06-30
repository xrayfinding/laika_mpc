#pragma once

#include <pronto_quadruped_commons/feet_contact_forces.h>
#include <laikago_robcogen/inverse_dynamics.h>
#include <laikago_robcogen/jsim.h>
#include <pronto_laikago_commons/feet_jacobians.hpp>
#include "laikago_robcogen/inertia_properties.h"
namespace laikago {

class FeetContactForces : public pronto::quadruped::FeetContactForces {

public:
    typedef pronto::quadruped::Vector3d Vector3d;
    typedef pronto::quadruped::JointState JointState;
    typedef pronto::quadruped::LegID LegID;
    using LegVectorMap = pronto::quadruped::LegDataMap<Vector3d>;

public:
    inline FeetContactForces() :
        inverse_dynamics_(inertia_prop_, motion_transf_),
        jsim_(inertia_prop_, force_transf_)
    {

    }

    inline Vector3d getFootGRF(const JointState& q,
                        const JointState& qd,
                        const JointState& tau,
                        const Quaterniond& orient,
                        const LegID& leg,
                        const JointState& qdd = JointState::Zero(),
                        const Vector3d& xd = Vector3d::Zero(),
                        const Vector3d& xdd = Vector3d::Zero(),
                        const Vector3d& omega = Vector3d::Zero(),
                        const Vector3d& omegad = Vector3d::Zero()) {
        Vector3d res;
        getFootGRF(q, qd, tau, orient, leg, res, qdd, xd, xdd, omega, omegad);
        return res;
    }

    bool getFootGRF(const JointState& q,
                    const JointState& qd,
                    const JointState& tau,
                    const Quaterniond& orient,
                    const LegID& leg,
                    Vector3d& foot_grf,
                    const JointState& qdd = JointState::Zero(),
                    const Vector3d& xd = Vector3d::Zero(),
                    const Vector3d& xdd = Vector3d::Zero(),
                    const Vector3d& omega = Vector3d::Zero(),
                    const Vector3d& omegad = Vector3d::Zero());

    inline bool getFeetGRF(const JointState& q,
                    const JointState& qd,
                    const JointState& tau,
                    const Quaterniond& orient,
                    LegVectorMap& feet_grf,
                    const JointState& qdd = JointState::Zero(),
                    const Vector3d& xd = Vector3d::Zero(),
                    const Vector3d& xdd = Vector3d::Zero(),
                    const Vector3d& omega = Vector3d::Zero(),
                    const Vector3d& omegad = Vector3d::Zero()) {
        bool res_lf = getFootGRF(q, qd, tau, orient, LegID::LF, feet_grf[LegID::LF], qdd, xd, xdd, omega, omegad);
        bool res_rf = getFootGRF(q, qd, tau, orient, LegID::RF, feet_grf[LegID::RF], qdd, xd, xdd, omega, omegad);
        bool res_lh = getFootGRF(q, qd, tau, orient, LegID::LH, feet_grf[LegID::LH], qdd, xd, xdd, omega, omegad);
        bool res_rh = getFootGRF(q, qd, tau, orient, LegID::RH, feet_grf[LegID::RH], qdd, xd, xdd, omega, omegad);

        return(res_lf && res_rf && res_lh && res_rh);
    }

    LegVectorMap getFeetGRF(const JointState& q,
                            const JointState& qd,
                            const JointState& tau,
                            const Quaterniond& orient,
                            const JointState& qdd = JointState::Zero(),
                            const Vector3d& xd = Vector3d::Zero(),
                            const Vector3d& xdd = Vector3d::Zero(),
                            const Vector3d& omega = Vector3d::Zero(),
                            const Vector3d& omegad = Vector3d::Zero()) {
        LegVectorMap res;
        getFeetGRF(q, qd, tau, orient, res, qdd, xd, xdd, omega, omegad);
        return res;
    }

    inline void setContactPoint(LegID leg, double foot_x, double foot_y){

    }

    bool getTorqueFromIDyn(const JointState& q,
                           const JointState& qd,
                           JointState& tau,
                           const Quaterniond& orient,
                           const LegID& leg,
                           const Vector3d& foot_grf,
                           const JointState& qdd = JointState::Zero(),
                           const Vector3d& xd = Vector3d::Zero(),
                           const Vector3d& xdd = Vector3d::Zero(),
                           const Vector3d& omega = Vector3d::Zero(),
                           const Vector3d& omegad = Vector3d::Zero());
    inline bool getTorqueFromIDyn(const JointState& q,
                                  const JointState& qd,
                                  JointState& tau,
                                  const Quaterniond& orient,
                                  const LegVectorMap& feet_grf,
                                  const JointState& qdd = JointState::Zero(),
                                  const Vector3d& xd = Vector3d::Zero(),
                                  const Vector3d& xdd = Vector3d::Zero(),
                                  const Vector3d& omega = Vector3d::Zero(),
                                  const Vector3d& omegad = Vector3d::Zero()){
        bool lf = getTorqueFromIDyn(q,qd,tau,orient,LegID::LF,feet_grf[LegID::LF],qdd,xd,xdd,omega,omegad);
        bool rf = getTorqueFromIDyn(q,qd,tau,orient,LegID::RF,feet_grf[LegID::RF],qdd,xd,xdd,omega,omegad);
        bool lh = getTorqueFromIDyn(q,qd,tau,orient,LegID::LH,feet_grf[LegID::LH],qdd,xd,xdd,omega,omegad);
        bool rh = getTorqueFromIDyn(q,qd,tau,orient,LegID::RH,feet_grf[LegID::RH],qdd,xd,xdd,omega,omegad);
        return  lf&rf&lh&rh;
    }
private:
    laikago::rcg::InertiaProperties inertia_prop_;
    laikago::rcg::MotionTransforms motion_transf_;
    laikago::rcg::ForceTransforms force_transf_;
    laikago::rcg::InverseDynamics inverse_dynamics_;
    laikago::rcg::JSIM jsim_;
    FeetJacobians feet_jacs_;
//    pronto::laikago::dyn::InertiaProperties inertia_prop_;
//    pronto::anymal::MotionTransforms motion_transf_;
//    pronto::anymal::ForceTransforms force_transf_;
//    pronto::anymal::dyn::InverseDynamics inverse_dynamics_;
//    pronto::anymal::dyn::JSIM jsim_;
//    FeetJacobians feet_jacs_;

};

}
