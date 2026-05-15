/* -------------------------------------------------------------------------- *
 *                               Simbody(tm)                                  *
 * -------------------------------------------------------------------------- *
 * This is part of the SimTK biosimulation toolkit originating from           *
 * Simbios, the NIH National Center for Physics-Based Simulation of           *
 * Biological Structures at Stanford, funded under the NIH Roadmap for        *
 * Medical Research, grant U54 GM072970. See https://simtk.org/home/simbody.  *
 *                                                                            *
 * Portions copyright (c) 2005-15 Stanford University and the Authors.        *
 * Authors: Michael Sherman                                                   *
 * Contributors:                                                              *
 *    Charles Schwieters (NIH): wrote the public domain IVM code from which   *
 *                              this was derived.                             *
 *    Peter Eastman wrote the Weld joint.                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/**@file
 * This file contains the implementations for RigidBodyNodes which have
 * no degrees of freedom -- Ground and Weld mobilizers. These cannot be
 * derived from the usual RigidBodyNodeSpec<dof> class because dof==0
 * is problematic there. Also, these can have very efficient
 * implementations here since they know they have no dofs.
 */
#include "RigidBodyNode_Weld.h"

#include "MobilizedBodyImpl.h"

// The Ground node is special because it doesn't need a mobilizer.
/*static*/ RigidBodyNode* RigidBodyNode::createGroundNode() {
    return new RBGroundBody();
}

RigidBodyNode* MobilizedBody::WeldImpl::createRigidBodyNode(UIndex& nextUSlot,
                                                            USquaredIndex& nextUSqSlot,
                                                            QIndex& nextQSlot) const {
    return new RBNodeWeld(getDefaultRigidBodyMassProperties(),
                          getDefaultInboardFrame(),
                          getDefaultOutboardFrame(),
                          nextUSlot,
                          nextUSqSlot,
                          nextQSlot);
}

RigidBodyNode* MobilizedBody::GroundImpl::createRigidBodyNode(UIndex& nextUSlot,
                                                              USquaredIndex& nextUSqSlot,
                                                              QIndex& nextQSlot) const {
    return RigidBodyNode::createGroundNode();
}
