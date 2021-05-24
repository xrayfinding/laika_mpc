#ifndef RCG__LAIKAGO_TRAITS_H_
#define RCG__LAIKAGO_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace laikago {
namespace rcg {
struct Traits {
    typedef typename laikago::rcg::ScalarTraits ScalarTraits;

    typedef typename laikago::rcg::JointState JointState;

    typedef typename laikago::rcg::JointIdentifiers JointID;
    typedef typename laikago::rcg::LinkIdentifiers  LinkID;

    typedef typename laikago::rcg::HomogeneousTransforms HomogeneousTransforms;
    typedef typename laikago::rcg::MotionTransforms MotionTransforms;
    typedef typename laikago::rcg::ForceTransforms ForceTransforms;

    typedef typename laikago::rcg::InertiaProperties InertiaProperties;
    typedef typename laikago::rcg::ForwardDynamics FwdDynEngine;
    typedef typename laikago::rcg::InverseDynamics InvDynEngine;
    typedef typename laikago::rcg::JSIM JSIM;

    static const int joints_count = laikago::rcg::jointsCount;
    static const int links_count  = laikago::rcg::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return laikago::rcg::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return laikago::rcg::orderedLinkIDs;
}

}
}

#endif
