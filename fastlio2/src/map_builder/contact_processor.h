#pragma once
#include "commons.h"
#include "ieskf.h"

class ContactProcessor
{
public:
    ContactProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void prepare(SyncPackage &package);

    void process(SyncPackage &package);

    void updateLossFunc(State &state, SharedState &share_data);

    bool hasLowState() const { return m_has_lowstate; }
    const LowStateData &latestLowState() const { return m_latest_lowstate; }

private:
    void cacheLowStates(SyncPackage &package);
    void cacheImus(SyncPackage &package);
    void updateContactState();
    void initializeNewContacts();
    void resetContactState();
    bool hasActiveContact() const;
    V3D gravityOffsetBody(const State &state) const;
    V3D footRelativePosition(const State &state, int foot_idx) const;
    V3D footRelativeVelocity(int foot_idx) const;
    V3D &footPosition(State &state, int foot_idx) const;
    const V3D &footPosition(const State &state, int foot_idx) const;

    Config m_config;
    std::shared_ptr<IESKF> m_kf;
    LowStateData m_latest_lowstate;
    V4D m_contact_state;
    V4D m_prev_contact_state;
    V3D m_latest_gyro = V3D::Zero();
    bool m_has_lowstate = false;
    bool m_has_imu = false;
};
