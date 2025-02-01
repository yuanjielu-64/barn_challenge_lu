/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */

#ifndef Antipatrea__GroupKeyRegion_HPP_
#define Antipatrea__GroupKeyRegion_HPP_

#include "Components/GroupKeyId.hpp"

namespace Antipatrea {

    class GroupKeyRegion : public GroupKeyId {
    public:
        GroupKeyRegion(void) : GroupKeyId() {
        }

        virtual ~GroupKeyRegion(void) {
        }

        virtual void SetContent(const GroupKey &key) {
            GroupKey::SetContent(key);
            SetId(dynamic_cast<const GroupKeyRegion &>(key).GetId());
        }

        virtual void SetContent(const Problem &prob) {
            GroupKeyId::SetContent(prob);
        }

        virtual void UpdateContent(const MPTreeVertex &v) {
            GroupKey::UpdateContent(v);
            SetId(v.GetRegion());
        }
    };

    ClassContainer(GroupKeyRegion, m_groupKeyRegion);
}

#endif
