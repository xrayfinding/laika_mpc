#ifndef RCG_LAIKAGO_MODEL_CONSTANTS_H_
#define RCG_LAIKAGO_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace laikago {
namespace rcg {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar tx_RF_HAA = 0.21934999525547028;
const Scalar ty_RF_HAA = -0.08749999850988388;
const Scalar ty_RF_HFE = -0.03700000047683716;
const Scalar tx_RF_KFE = 0.25;
const Scalar tx_LF_HAA = 0.21934999525547028;
const Scalar ty_LF_HAA = 0.08749999850988388;
const Scalar ty_LF_HFE = 0.03700000047683716;
const Scalar tx_LF_KFE = 0.25;
const Scalar tx_RH_HAA = -0.21934999525547028;
const Scalar ty_RH_HAA = -0.08749999850988388;
const Scalar ty_RH_HFE = -0.03700000047683716;
const Scalar tx_RH_KFE = 0.25;
const Scalar tx_LH_HAA = -0.21934999525547028;
const Scalar ty_LH_HAA = 0.08749999850988388;
const Scalar ty_LH_HFE = 0.03700000047683716;
const Scalar tx_LH_KFE = 0.25;
const Scalar tx_RF_FOOT = 0.25;
const Scalar tx_LF_FOOT = 0.25;
const Scalar tx_RH_FOOT = 0.25;
const Scalar tx_LH_FOOT = 0.25;
const Scalar m_base = 13.734000205993652;
const Scalar comx_base = 0.002283999929204583;
const Scalar comy_base = -4.099999932805076E-5;
const Scalar comz_base = 0.02516300044953823;
const Scalar ix_base = 0.08214599639177322;
const Scalar ixy_base = -3.0499999411404133E-4;
const Scalar ixz_base = -0.0011289999820291996;
const Scalar iy_base = 0.25945401191711426;
const Scalar iyz_base = 6.0999998822808266E-5;
const Scalar iz_base = 0.25464099645614624;
const Scalar m_RF_HIP = 1.0959999561309814;
const Scalar comx_RF_HIP = -8.640000014565885E-4;
const Scalar comy_RF_HIP = 0.00813400000333786;
const Scalar comz_RF_HIP = -0.001567999948747456;
const Scalar ix_RF_HIP = 9.399999980814755E-4;
const Scalar ixy_RF_HIP = -1.1000000085914508E-5;
const Scalar ixz_RF_HIP = 6.000000212225132E-6;
const Scalar iy_RF_HIP = 9.869999485090375E-4;
const Scalar iyz_RF_HIP = -1.8999999156221747E-5;
const Scalar iz_RF_HIP = 8.950000046752393E-4;
const Scalar m_RF_THIGH = 1.527999997138977;
const Scalar comx_RF_THIGH = 0.031996000558137894;
const Scalar comy_RF_THIGH = 4.820000031031668E-4;
const Scalar comz_RF_THIGH = -0.020010000094771385;
const Scalar ix_RF_THIGH = 0.0023950000759214163;
const Scalar ixy_RF_THIGH = 3.400000059627928E-5;
const Scalar ixz_RF_THIGH = -8.929999894462526E-4;
const Scalar iy_RF_THIGH = 0.012091999873518944;
const Scalar iyz_RF_THIGH = -2.499999936844688E-5;
const Scalar iz_RF_THIGH = 0.010844999924302101;
const Scalar m_RF_SHANK = 0.3009999990463257;
const Scalar comx_RF_SHANK = 0.14861999452114105;
const Scalar comy_RF_SHANK = 0.0017579999985173345;
const Scalar comz_RF_SHANK = -3.0499999411404133E-4;
const Scalar ix_RF_SHANK = 5.2999999752501026E-5;
const Scalar ixy_RF_SHANK = 6.500000017695129E-5;
const Scalar ixz_RF_SHANK = -6.000000212225132E-6;
const Scalar iy_RF_SHANK = 0.013617999851703644;
const Scalar iz_RF_SHANK = 0.0136329997330904;
const Scalar m_LF_HIP = 1.0959999561309814;
const Scalar comx_LF_HIP = -8.640000014565885E-4;
const Scalar comy_LF_HIP = -0.00813400000333786;
const Scalar comz_LF_HIP = -0.001567999948747456;
const Scalar ix_LF_HIP = 9.399999980814755E-4;
const Scalar ixy_LF_HIP = 1.1000000085914508E-5;
const Scalar ixz_LF_HIP = -3.000000106112566E-6;
const Scalar iy_LF_HIP = 9.869999485090375E-4;
const Scalar iyz_LF_HIP = 1.8999999156221747E-5;
const Scalar iz_LF_HIP = 8.950000046752393E-4;
const Scalar m_LF_THIGH = 1.527999997138977;
const Scalar comx_LF_THIGH = 0.031996000558137894;
const Scalar comy_LF_THIGH = 4.820000031031668E-4;
const Scalar comz_LF_THIGH = 0.020010000094771385;
const Scalar ix_LF_THIGH = 0.0023950000759214163;
const Scalar ixy_LF_THIGH = 1.2999999853491317E-5;
const Scalar ixz_LF_THIGH = 8.929999894462526E-4;
const Scalar iy_LF_THIGH = 0.012091999873518944;
const Scalar iyz_LF_THIGH = 2.499999936844688E-5;
const Scalar iz_LF_THIGH = 0.010844999924302101;
const Scalar m_LF_SHANK = 0.3009999990463257;
const Scalar comx_LF_SHANK = 0.14861999452114105;
const Scalar comy_LF_SHANK = 0.0017579999985173345;
const Scalar comz_LF_SHANK = -3.0499999411404133E-4;
const Scalar ix_LF_SHANK = 5.2999999752501026E-5;
const Scalar ixy_LF_SHANK = 6.500000017695129E-5;
const Scalar ixz_LF_SHANK = -6.000000212225132E-6;
const Scalar iy_LF_SHANK = 0.013617999851703644;
const Scalar iz_LF_SHANK = 0.0136329997330904;
const Scalar m_RH_HIP = 1.0959999561309814;
const Scalar comx_RH_HIP = -8.640000014565885E-4;
const Scalar comy_RH_HIP = 0.00813400000333786;
const Scalar comz_RH_HIP = 0.001567999948747456;
const Scalar ix_RH_HIP = 9.399999980814755E-4;
const Scalar ixy_RH_HIP = -1.1000000085914508E-5;
const Scalar ixz_RH_HIP = -6.000000212225132E-6;
const Scalar iy_RH_HIP = 9.869999485090375E-4;
const Scalar iyz_RH_HIP = 1.8999999156221747E-5;
const Scalar iz_RH_HIP = 8.950000046752393E-4;
const Scalar m_RH_THIGH = 1.527999997138977;
const Scalar comx_RH_THIGH = 0.031996000558137894;
const Scalar comy_RH_THIGH = 4.820000031031668E-4;
const Scalar comz_RH_THIGH = -0.020010000094771385;
const Scalar ix_RH_THIGH = 0.0023950000759214163;
const Scalar ixy_RH_THIGH = 3.400000059627928E-5;
const Scalar ixz_RH_THIGH = -8.929999894462526E-4;
const Scalar iy_RH_THIGH = 0.012091999873518944;
const Scalar iyz_RH_THIGH = -2.499999936844688E-5;
const Scalar iz_RH_THIGH = 0.010844999924302101;
const Scalar m_RH_SHANK = 0.3009999990463257;
const Scalar comx_RH_SHANK = 0.14861999452114105;
const Scalar comy_RH_SHANK = 0.0017579999985173345;
const Scalar comz_RH_SHANK = -3.0499999411404133E-4;
const Scalar ix_RH_SHANK = 5.2999999752501026E-5;
const Scalar ixy_RH_SHANK = 6.500000017695129E-5;
const Scalar ixz_RH_SHANK = -6.000000212225132E-6;
const Scalar iy_RH_SHANK = 0.013617999851703644;
const Scalar iz_RH_SHANK = 0.0136329997330904;
const Scalar m_LH_HIP = 1.0959999561309814;
const Scalar comx_LH_HIP = -8.640000014565885E-4;
const Scalar comy_LH_HIP = -0.00813400000333786;
const Scalar comz_LH_HIP = 0.001567999948747456;
const Scalar ix_LH_HIP = 9.399999980814755E-4;
const Scalar ixy_LH_HIP = 1.1000000085914508E-5;
const Scalar ixz_LH_HIP = 3.000000106112566E-6;
const Scalar iy_LH_HIP = 9.869999485090375E-4;
const Scalar iyz_LH_HIP = -1.8999999156221747E-5;
const Scalar iz_LH_HIP = 8.950000046752393E-4;
const Scalar m_LH_THIGH = 1.527999997138977;
const Scalar comx_LH_THIGH = 0.031996000558137894;
const Scalar comy_LH_THIGH = 4.820000031031668E-4;
const Scalar comz_LH_THIGH = 0.020010000094771385;
const Scalar ix_LH_THIGH = 0.0023950000759214163;
const Scalar ixy_LH_THIGH = 1.2999999853491317E-5;
const Scalar ixz_LH_THIGH = 8.929999894462526E-4;
const Scalar iy_LH_THIGH = 0.012091999873518944;
const Scalar iyz_LH_THIGH = 2.499999936844688E-5;
const Scalar iz_LH_THIGH = 0.010844999924302101;
const Scalar m_LH_SHANK = 0.3009999990463257;
const Scalar comx_LH_SHANK = 0.14861999452114105;
const Scalar comy_LH_SHANK = 0.0017579999985173345;
const Scalar comz_LH_SHANK = -3.0499999411404133E-4;
const Scalar ix_LH_SHANK = 5.2999999752501026E-5;
const Scalar ixy_LH_SHANK = 6.500000017695129E-5;
const Scalar ixz_LH_SHANK = -6.000000212225132E-6;
const Scalar iy_LH_SHANK = 0.013617999851703644;
const Scalar iz_LH_SHANK = 0.0136329997330904;

}
}
#endif
