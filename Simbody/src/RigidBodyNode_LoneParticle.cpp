#include "RigidBodyNode_LoneParticle.h"

#include "MobilizedBodyImpl.h"
#include "RigidBodyNodeSpec_Translation.h"

RigidBodyNode* MobilizedBody::TranslationImpl::createRigidBodyNode(UIndex& nextUSlot,
                                                                   USquaredIndex& nextUSqSlot,
                                                                   QIndex& nextQSlot) const {
    if (!hasChildren && getMyParentMobilizedBodyIndex() == 0 && !isReversed()
        && getDefaultInboardFrame().p() == 0 && getDefaultInboardFrame().R() == Mat33(1)
        && getDefaultOutboardFrame().p() == 0 && getDefaultOutboardFrame().R() == Mat33(1)) {
        // This satisfies all the requirements to use RBNodeLoneParticle.

        return new RBNodeLoneParticle(getDefaultRigidBodyMassProperties(), nextUSlot, nextUSqSlot, nextQSlot);
    }

    // Use RBNodeTranslate for the general case.

    bool noX_MB = (getDefaultOutboardFrame().p() == 0 && getDefaultOutboardFrame().R() == Mat33(1));
    bool noR_PF = (getDefaultInboardFrame().R() == Mat33(1));
    if (noX_MB) {
        if (noR_PF) {
            return new RBNodeTranslate<true, true>(getDefaultRigidBodyMassProperties(),
                                                   getDefaultInboardFrame(),
                                                   getDefaultOutboardFrame(),
                                                   isReversed(),
                                                   nextUSlot,
                                                   nextUSqSlot,
                                                   nextQSlot);
        } else {
            return new RBNodeTranslate<true, false>(getDefaultRigidBodyMassProperties(),
                                                    getDefaultInboardFrame(),
                                                    getDefaultOutboardFrame(),
                                                    isReversed(),
                                                    nextUSlot,
                                                    nextUSqSlot,
                                                    nextQSlot);
        }
    } else {
        if (noR_PF) {
            return new RBNodeTranslate<false, true>(getDefaultRigidBodyMassProperties(),
                                                    getDefaultInboardFrame(),
                                                    getDefaultOutboardFrame(),
                                                    isReversed(),
                                                    nextUSlot,
                                                    nextUSqSlot,
                                                    nextQSlot);
        } else {
            return new RBNodeTranslate<false, false>(getDefaultRigidBodyMassProperties(),
                                                     getDefaultInboardFrame(),
                                                     getDefaultOutboardFrame(),
                                                     isReversed(),
                                                     nextUSlot,
                                                     nextUSqSlot,
                                                     nextQSlot);
        }
    }
}
