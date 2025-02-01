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

#ifndef Antipatrea__DecompositionPRM_HPP_
#define Antipatrea__DecompositionPRM_HPP_

#include "Components/Decomposition.hpp"
#include "Components/Constants.hpp"
#include "Utils/ProximityDefault.hpp"
#include "Utils/HashFn.hpp"
#include <unordered_set>

namespace Antipatrea {

    class DecompositionPRM : public Decomposition {
    public:
        DecompositionPRM(void) : Decomposition(),
                                 m_nrNeighbors(Constants::PRM_NR_NEIGHBORS),
                                 m_batchSize(Constants::PRM_BATCH_SIZE),
                                 m_probAllowCycles(Constants::PRM_PROBABILITY_ALLOW_CYCLES),
                                 m_oneStepDistance(Constants::PRM_ONE_STEP_DISTANCE),
                                 m_stopWhenConnected(false),
                                 m_nrRemainingToCompleteBatch(-1),
                                 m_maxNrVertices(Constants::PRM_MAX_NR_VERTICES) {
            m_proximity.m_distFn = ProximityDistanceFn;
            m_proximity.m_distFnData = this;
        }

        virtual ~DecompositionPRM(void) {
        }

        virtual int GetMaxNrVertices(void) const {
            return m_maxNrVertices;
        }

        virtual int GetNrNeighbors(void) const {
            return m_nrNeighbors;
        }

        virtual int GetBatchSize(void) const {
            return m_batchSize;
        }

        virtual double GetProbabilityAllowCycles(void) const {
            return m_probAllowCycles;
        }

        virtual double GetOneStepDistance(void) const {
            return m_oneStepDistance;
        }

        virtual bool GetStopWhenConnected(void) const {
            return m_stopWhenConnected;
        }

        virtual void SetMaxNrVertices(const int n) {
            m_maxNrVertices = n;
        }

        virtual void SetNrNeighbors(const int k) {
            m_nrNeighbors = k;
        }

        virtual void SetBatchSize(const int n) {
            m_batchSize = n;
        }

        virtual void SetProbabilityAllowCycles(const double p) {
            m_probAllowCycles = p;
        }

        virtual void SetOneStepDistance(const double d) {
            m_oneStepDistance = d;
        }

        virtual void SetStopWhenConnected(const bool f) {
            m_stopWhenConnected = f;
        }

        virtual void SetupFromParams(Params &params) {
            Decomposition::SetupFromParams(params);
            int a = GetMaxNrVertices();
            SetMaxNrVertices(params.GetValueAsInt(Constants::KW_MaxNrVertices, GetMaxNrVertices()));
            SetNrNeighbors(params.GetValueAsInt(Constants::KW_NrNeighbors, GetNrNeighbors()));
            SetBatchSize(params.GetValueAsDouble(Constants::KW_BatchSize, GetBatchSize()));
            SetProbabilityAllowCycles(
                    params.GetValueAsDouble(Constants::KW_ProbabilityAllowCycles, GetProbabilityAllowCycles()));
            SetOneStepDistance(params.GetValueAsDouble(Constants::KW_OneStepDistance, GetOneStepDistance()));
            SetStopWhenConnected(params.GetValueAsBool(Constants::KW_StopWhenConnected, GetStopWhenConnected()));
        }

        virtual void Construct(const double tmax);

        virtual bool Construct();

        virtual Id LocateRegion(const double cfg[]);

    protected:
        virtual void AddGoals(void);

        virtual Id AddRegion(const double cfg[]);

        virtual int AddRegions(const int nrRegions, const double tmax);

        virtual int ConnectRegions(const double tmax);

        virtual void ConnectRegion(const Id key);

        virtual bool GenerateEdge(const Id key1, const Id key2, const double d);

        static double ProximityDistanceFn(const Id key1, const Id key2, DecompositionPRM *prm);

        int m_nrNeighbors;
        int m_batchSize;
        double m_probAllowCycles;
        double m_oneStepDistance;
        bool m_stopWhenConnected;
        int m_nrRemainingToCompleteBatch;
        int m_maxNrVertices;
        ProximityDefault<Id, DecompositionPRM *> m_proximity;
        std::vector<double> m_proximityAuxCfg;
        std::unordered_set<Id> m_regionsToBeConnected;
        std::unordered_set<std::pair<Id, Id>, HashStruct<std::pair<Id, Id>>> m_attempts;
    };
}

#endif
