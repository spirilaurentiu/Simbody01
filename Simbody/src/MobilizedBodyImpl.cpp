#include "MobilizedBodyImpl.h"

namespace SimTK {

//------------------------------------------------------------------------------
// Constructors / Destructor
//------------------------------------------------------------------------------

MobilizedBodyImpl::MobilizedBodyImpl(MobilizedBody::Direction d)
    : defaultLockLevel(Motion::NoLevel)
    , reversed(d == MobilizedBody::Reverse)
    , myMatterSubsystemRep(0)
    , myLevel(-1)
    , myRBnode(0)
    , hasChildren(false) {
}

MobilizedBodyImpl::MobilizedBodyImpl(const MobilizedBodyImpl& src) {
    *this = src;
    myRBnode = 0;
}

//------------------------------------------------------------------------------
// Direction / locking
//------------------------------------------------------------------------------

void MobilizedBodyImpl::setDirection(MobilizedBody::Direction d) {
    const bool wantReversed = (d == MobilizedBody::Reverse);
    if (wantReversed != reversed) {
        invalidateTopologyCache();
        reversed = wantReversed;
    }
}

void MobilizedBodyImpl::lockByDefault(Motion::Level level) {
    invalidateTopologyCache();
    defaultLockLevel = level;
}

Motion::Level MobilizedBodyImpl::getLockByDefaultLevel() const {
    return defaultLockLevel;
}

//------------------------------------------------------------------------------
// Decorative geometry
//------------------------------------------------------------------------------

int MobilizedBodyImpl::addOutboardDecoration(const Transform& X_MD, const DecorativeGeometry& g) {
    throw std::runtime_error("addOutboardDecoration() not implemented");
    return -1;
    // const int nxt = (int)outboardGeometry.size();
    // outboardGeometry.push_back(g); // make a new copy
    // // Combine the placement frame and the transform already in the geometry
    // // so we end up with geometry expressed directly in the M frame.
    // outboardGeometry.back().setTransform(X_MD * g.getTransform());
    // // Record the assigned ordinal (not in the same ordinal space as
    // // body decorations).
    // outboardGeometry.back().setIndexOnBody(nxt);
    // return nxt;
}

int MobilizedBodyImpl::addInboardDecoration(const Transform& X_FD, const DecorativeGeometry& g) {
    throw std::runtime_error("addInboardDecoration() not implemented");
    return -1;
    // const int nxt = (int)inboardGeometry.size();
    // inboardGeometry.push_back(g); // make a new copy
    // // Combine the placement frame and the transform already in the geometry
    // // so we end up with geometry expressed directly in the F frame.
    // inboardGeometry.back().setTransform(X_FD * g.getTransform());
    // // Record the assigned ordinal (not in the same ordinal space as
    // // body decorations).
    // inboardGeometry.back().setIndexOnBody(nxt);
    // return nxt;
}

//------------------------------------------------------------------------------
// State accessors — body properties
//------------------------------------------------------------------------------

const MassProperties& MobilizedBodyImpl::getBodyMassProperties(const State& s) const {
    // TODO: these should come from the state if the body has variable mass props
    const SBInstanceVars& iv = getMyMatterSubsystemRep().getInstanceVars(s);
    return getMyRigidBodyNode().getMassProperties_OB_B();
}

const SpatialInertia& MobilizedBodyImpl::getBodySpatialInertiaInGround(const State& s) const {
    const SBTreePositionCache& tpc = getMyMatterSubsystemRep().getTreePositionCache(s);
    return getMyRigidBodyNode().getMk_G(tpc);
}

const Transform& MobilizedBodyImpl::getInboardFrame(const State& s) const {
    // TODO: these should come from the state if the mobilizer has variable frames
    const SBInstanceVars& iv = getMyMatterSubsystemRep().getInstanceVars(s);
    return getMyRigidBodyNode().getX_PF();
}

const Transform& MobilizedBodyImpl::getOutboardFrame(const State& s) const {
    // TODO: these should come from the state if the mobilizer has variable frames
    const SBInstanceVars& iv = getMyMatterSubsystemRep().getInstanceVars(s);
    return getMyRigidBodyNode().getX_BM();
}

void MobilizedBodyImpl::setInboardFrame(State& s, const Transform& X_PF) const {
    assert(!"setInboardFrame(s) not implemented yet");
}

void MobilizedBodyImpl::setOutboardFrame(State& s, const Transform& X_BM) const {
    assert(!"setOutboardFrame(s) not implemented yet");
}

const Transform& MobilizedBodyImpl::getBodyTransform(const State& s) const {
    const SBTreePositionCache& pc = getMyMatterSubsystemRep().getTreePositionCache(s);
    return getMyRigidBodyNode().getX_GB(pc);
}

const SpatialVec& MobilizedBodyImpl::getBodyVelocity(const State& s) const {
    const SBTreeVelocityCache& vc = getMyMatterSubsystemRep().getTreeVelocityCache(s);
    return getMyRigidBodyNode().getV_GB(vc);
}

const SpatialVec& MobilizedBodyImpl::getBodyAcceleration(const State& s) const {
    const SBTreeAccelerationCache& ac = getMyMatterSubsystemRep().getTreeAccelerationCache(s);
    return getMyRigidBodyNode().getA_GB(ac);
}

const Transform& MobilizedBodyImpl::getMobilizerTransform(const State& s) const {
    const SBTreePositionCache& pc = getMyMatterSubsystemRep().getTreePositionCache(s);
    return getMyRigidBodyNode().getX_FM(pc);
}

const SpatialVec& MobilizedBodyImpl::getMobilizerVelocity(const State& s) const {
    const SBTreeVelocityCache& vc = getMyMatterSubsystemRep().getTreeVelocityCache(s);
    return getMyRigidBodyNode().getV_FM(vc);
}

SpatialVec MobilizedBodyImpl::getHCol(const State& s, MobilizerUIndex ux) const {
    const SBTreePositionCache& pc = getMyMatterSubsystemRep().getTreePositionCache(s);
    return getMyRigidBodyNode().getHCol(pc, ux);
}

SpatialVec MobilizedBodyImpl::getH_FMCol(const State& s, MobilizerUIndex ux) const {
    const SBTreePositionCache& pc = getMyMatterSubsystemRep().getTreePositionCache(s);
    return getMyRigidBodyNode().getH_FMCol(pc, ux);
}

//------------------------------------------------------------------------------
// Topology cache
//------------------------------------------------------------------------------

void MobilizedBodyImpl::invalidateTopologyCache() const {
    delete myRBnode;
    myRBnode = 0;
    if (myMatterSubsystemRep) {
        myMatterSubsystemRep->invalidateSubsystemTopologyCache();
    }
}

const RigidBodyNode& MobilizedBodyImpl::getMyRigidBodyNode() const {
    SimTK_ASSERT(myRBnode && myMatterSubsystemRep,
                 "An operation on a MobilizedBody was illegal because realizeTopology() has "
                 "not been performed on the containing Subsystem since the last topological change.");
    return *myRBnode;
}

//------------------------------------------------------------------------------
// Default-frame / body accessors
//------------------------------------------------------------------------------

const Body& MobilizedBodyImpl::getBody() const {
    return theBody;
}

const Transform& MobilizedBodyImpl::getDefaultInboardFrame() const {
    return defaultInboardFrame;
}

const Transform& MobilizedBodyImpl::getDefaultOutboardFrame() const {
    return defaultOutboardFrame;
}

const MassProperties& MobilizedBodyImpl::getDefaultRigidBodyMassProperties() const {
    return theBody.getDefaultRigidBodyMassProperties();
}

bool MobilizedBodyImpl::isReversed() const {
    return reversed;
}

//------------------------------------------------------------------------------
// Motion management
//------------------------------------------------------------------------------

void MobilizedBodyImpl::adoptMotion(Motion& ownerHandle) {
    SimTK_ERRCHK(!hasMotion(),
                 "MobilizedBody::adoptMotion()",
                 "This MobilizedBody already has a Motion object associated with it.\n"
                 "Use clearMotion() first to replace an existing Motion object.");
    ownerHandle.disown(motion); // transfer ownership to handle "motion"
    invalidateTopologyCache();

    // Tell the Motion that it belongs to this MobilizedBody.
    motion.updImpl().setMobilizedBodyImpl(this);
}

void MobilizedBodyImpl::clearMotion() {
    motion.clearHandle();
    invalidateTopologyCache();
}

bool MobilizedBodyImpl::hasMotion() const {
    return !motion.isEmptyHandle();
}

const Motion& MobilizedBodyImpl::getMotion() const {
    SimTK_ERRCHK(!motion.isEmptyHandle(),
                 "MobilizedBody::getMotion()",
                 "There is no Motion object associated with this MobilizedBody.");
    return motion;
}

//------------------------------------------------------------------------------
// Matter subsystem accessors
//------------------------------------------------------------------------------

const SimbodyMatterSubsystem& MobilizedBodyImpl::getMySimbodyMatterSubsystem() const {
    return getMyMatterSubsystemRep().getMySimbodyMatterSubsystemHandle();
}

const SimbodyMatterSubsystemRep& MobilizedBodyImpl::getMyMatterSubsystemRep() const {
    SimTK_ASSERT(myMatterSubsystemRep,
                 "An operation was illegal because a MobilizedBody was not in a Subsystem.");
    return *myMatterSubsystemRep;
}

SimbodyMatterSubsystemRep& MobilizedBodyImpl::updMyMatterSubsystemRep() {
    SimTK_ASSERT(myMatterSubsystemRep,
                 "An operation was illegal because a MobilizedBody was not in a Subsystem.");
    return *myMatterSubsystemRep;
}

const SBModelPerMobodInfo& MobilizedBodyImpl::getMyModelInfo(const State& s) const {
    const SimbodyMatterSubsystemRep& matterRep = getMyMatterSubsystemRep();
    const SBModelCache& mc = matterRep.getModelCache(s);
    const SBModelPerMobodInfo& info = mc.getMobodModelInfo(myMobilizedBodyIndex);
    return info;
}

const SBInstancePerMobodInfo& MobilizedBodyImpl::getMyInstanceInfo(const State& s) const {
    const SimbodyMatterSubsystemRep& matterRep = getMyMatterSubsystemRep();
    const SBInstanceCache& ic = matterRep.getInstanceCache(s);
    const SBInstancePerMobodInfo& info = ic.getMobodInstanceInfo(myMobilizedBodyIndex);
    return info;
}

const MobilizedBody& MobilizedBodyImpl::getMyHandle() const {
    const MobilizedBody& mobod = getMyMatterSubsystemRep().getMobilizedBody(getMyMobilizedBodyIndex());
    SimTK_ASSERT(&mobod.getImpl() == this,
                 "A MobilizedBodyImpl's handle didn't refer back to the same Impl!");
    return mobod;
}

MobilizedBody& MobilizedBodyImpl::updMyHandle() {
    MobilizedBody& mobod = updMyMatterSubsystemRep().updMobilizedBody(getMyMobilizedBodyIndex());
    SimTK_ASSERT(&mobod.getImpl() == this,
                 "A MobilizedBodyImpl's handle didn't refer back to the same Impl!");
    return mobod;
}

//------------------------------------------------------------------------------
// Index / level accessors
//------------------------------------------------------------------------------

MobilizedBodyIndex MobilizedBodyImpl::getMyMobilizedBodyIndex() const {
    assert(myMobilizedBodyIndex.isValid());
    return myMobilizedBodyIndex;
}

MobilizedBodyIndex MobilizedBodyImpl::getMyParentMobilizedBodyIndex() const {
    assert(myParentIndex.isValid());
    return myParentIndex;
}

MobilizedBodyIndex MobilizedBodyImpl::getMyBaseBodyMobilizedBodyIndex() const {
    assert(myBaseBodyIndex.isValid());
    return myBaseBodyIndex;
}

int MobilizedBodyImpl::getMyLevel() const {
    assert(myLevel >= 0);
    return myLevel;
}

bool MobilizedBodyImpl::isInSubsystem() const {
    return myMatterSubsystemRep != 0;
}

void MobilizedBodyImpl::setMyMatterSubsystem(SimbodyMatterSubsystem& matter,
                                             MobilizedBodyIndex parentIndex,
                                             MobilizedBodyIndex index) {
    // If the subsystem is already SimbodyMatterSubsystemRepset it must be the same one.
    assert(!myMatterSubsystemRep || myMatterSubsystemRep == &matter.getRep());
    myMatterSubsystemRep = &matter.updRep();

    assert(index.isValid());
    assert(parentIndex.isValid() || index == GroundIndex);

    myParentIndex = parentIndex; // invalid if this is Ground
    myMobilizedBodyIndex = index;

    if (index != GroundIndex) {
        MobilizedBody& parent = matter.updMobilizedBody(parentIndex);
        myLevel = parent.getLevelInMultibodyTree() + 1;
        myBaseBodyIndex =
            (myLevel == 1 ? myMobilizedBodyIndex : parent.getBaseMobilizedBody().getMobilizedBodyIndex());
        parent.updImpl().hasChildren = true;
    } else {
        myLevel = 0;
        myBaseBodyIndex = GroundIndex;
    }
}

//------------------------------------------------------------------------------
// Private helpers
//------------------------------------------------------------------------------

void MobilizedBodyImpl::appendTopologicalBodyGeometry(Array_<DecorativeGeometry>& geom) const {
    throw std::runtime_error("appendTopologicalBodyGeometry() not implemented");
    // getBody().getRep().appendDecorativeGeometry(getMyMobilizedBodyIndex(), geom);
}

void MobilizedBodyImpl::appendTopologicalMobilizerGeometry(const Transform& X_BM,
                                                           const Transform& X_PF,
                                                           Array_<DecorativeGeometry>& geom) const {
    throw std::runtime_error("appendTopologicalMobilizerGeometry() not implemented.");
    // for (int i = 0; i < (int)outboardGeometry.size(); ++i) {
    //     geom.push_back(outboardGeometry[i]);
    //     geom.back()
    //         .setBodyId(getMyMobilizedBodyIndex())
    //         .setTransform(X_BM * outboardGeometry[i].getTransform());
    // }
    // for (int i = 0; i < (int)inboardGeometry.size(); ++i) {
    //     geom.push_back(inboardGeometry[i]);
    //     geom.back()
    //         .setBodyId(getMyParentMobilizedBodyIndex())
    //         .setTransform(X_PF * inboardGeometry[i].getTransform());
    // }
}

} // namespace SimTK
