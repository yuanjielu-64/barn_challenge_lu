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

#ifndef Antipatrea__Simulator_HPP_
#define Antipatrea__Simulator_HPP_

#include "Components/Allocator.hpp"
#include "Scene.hpp"
#include "Utils/Algebra.hpp"

namespace Antipatrea {

    class Simulator
            : public Component, public SceneContainer {
    public:
        Simulator(void);

        virtual ~Simulator(void);

        virtual void Info(void) const {
            Component::Info();
            Logger::m_out << " Scene = " << Name(GetScene()) << std::endl;
        }

        virtual bool CheckSetup(void) const {
            return Component::CheckSetup() && GetScene() && GetScene()->CheckSetup();
        }

        // allocators
        virtual const Allocator *GetStateAllocator(void) const {
            return &m_stateAllocator;
        }

        virtual const Allocator *GetControlAllocator(void) const {
            return &m_controlAllocator;
        }

        virtual const Allocator *GetCfgAllocator(void) const {
            return &m_cfgAllocator;
        }

        virtual const Allocator *GetPositionAllocator(void) const {
            return &m_positionAllocator;
        }

        virtual Allocator *GetStateAllocator(void) {
            return &m_stateAllocator;
        }

        virtual Allocator *GetControlAllocator(void) {
            return &m_controlAllocator;
        }

        virtual Allocator *GetCfgAllocator(void) {
            return &m_cfgAllocator;
        }

        virtual Allocator *GetPositionAllocator(void) {
            return &m_positionAllocator;
        }

        // parameter setup
        virtual void SetTimeStep(const double dt) {
            m_dt = dt;
        }

        virtual void SetMinDistanceOneStep(const double d) {
            m_minDistanceOneStep = d;
        }

        virtual void SetMaxDistanceOneStep(const double d) {
            m_maxDistanceOneStep = d;
        }

        virtual void SetDistanceReachedSteerPosition(const double d) {
            m_distanceReachedSteerPosition = d;
        }

        virtual void SetStopDistance(const double d) {
            m_stopDistance = d;
        }

        virtual void SetStopExponent(const double d) {
            m_stopExponent = d;
        }

        virtual void SetVelocityScaleSampling(const double scale) {
            m_velocityScaleSampling = scale;
        }

        virtual void SetVelocityScaleConversion(const double scale) {
            m_velocityScaleConversion = scale;
        }

        virtual void SetVelocityScaleMin(const double scale) {
            m_velocityScaleMin = scale;
        }

        virtual void SetDrawPositionRadius(const double r) {
            m_drawPositionRadius = r;
        }

        virtual void SetupFromParams(Params &params);

        virtual void SetupFromParams(Params &params, Robot_config &robot){};

        virtual void CompleteSetup(void);

        // parameter access
        virtual double GetTimeStep(void) const {
            return m_dt;
        }

        virtual double GetMinDistanceOneStep(void) const {
            return m_minDistanceOneStep;
        }

        virtual double GetMaxDistanceOneStep(void) const {
            return m_maxDistanceOneStep;
        }

        virtual double GetDistanceReachedSteerPosition(void) const {
            return m_distanceReachedSteerPosition;
        }

        virtual double GetStopDistance(void) const {
            return m_stopDistance;
        }

        virtual double GetStopExponent(void) {
            return m_stopExponent;
        }

        virtual double GetVelocityScaleSampling(void) const {
            return m_velocityScaleSampling;
        }

        virtual double GetVelocityScaleConversion(void) const {
            return m_velocityScaleConversion;
        }

        virtual double GetVelocityScaleMin(void) const {
            return m_velocityScaleMin;
        }

        virtual double GetDrawPositionRadius(void) const {
            return m_drawPositionRadius;
        }

        // state functionality
        virtual void SetState(const double s[]) {
            if (s != m_currState)
                GetStateAllocator()->Copy(m_currState, s);
        }

        virtual void GetState(double s[]) {
            GetStateAllocator()->Copy(s, m_currState);
        }

        virtual void SampleState(double s[]) = 0;

        virtual bool IsWithinLimitsState(void) = 0;

        virtual void AddToMeshState(TriMesh &tmesh);

        virtual bool IsObstacleCollisionFreeState(void);

        virtual bool IsSelfCollisionFreeState(void) {
            return true;
        }

        virtual bool IsValidState(void) {
            return IsWithinLimitsState() && IsObstacleCollisionFreeState() && IsSelfCollisionFreeState();
        }

        virtual double GetMaxVelocity(void) const = 0;

        virtual double GetMinVelocity(void) const = 0;

        virtual double GetMaxAngularVelocity(void) const = 0;

        virtual double GetMinAngularVelocity(void) const = 0;

        virtual double GetMaxAcceleration(void) const = 0;

        virtual double GetMinAcceleration(void) const = 0;

        virtual double GetMaxAngularAcceleration(void) const = 0;

        virtual double GetMinAngularAcceleration(void) const = 0;


        virtual double GetVelocity(void) {
            return 0.0;
        }

        virtual double GetAcceleration(void) {
            return 0.0;
        }

        virtual double GetAngularVelocity(void){
            return 0.0;
        }

        // control functionality
        virtual void SetControl(const double u[]) {
            if (m_currControl != u)
                GetControlAllocator()->Copy(m_currControl, u);
        }

        virtual void GetControl(double u[]) {
            GetControlAllocator()->Copy(u, m_currControl);
        }

        virtual void SampleControl(double u[]) = 0;

        virtual void StartSteerToPosition(const double target[]) = 0;

        virtual void StartToPosition(double v, double w) = 0;

        virtual void SteerToPosition(const double target[], const bool stop = false) = 0;

        virtual void ToPosition(double v, double w) = 0 ;

        virtual bool HasReachedSteerPosition(const double target[]) {
            return DistanceStatePosition(m_currState, target) <= GetDistanceReachedSteerPosition();
        }

        // cfg functionality
        virtual void GetCfg(double cfg[]) {
            GetCfgState(m_currState, cfg);
        }

        virtual void SampleCfg(double cfg[]) = 0;

        virtual void AddToMeshCfg(TriMesh &tmesh, const double cfg[]) const = 0;

        virtual bool IsWithinLimitsCfg(const double cfg[]) = 0;

        virtual bool IsObstacleCollisionFreeCfg(const double cfg[]);

        virtual bool IsSelfCollisionFreeCfg(const double cfg[]) {
            return true;
        }

        virtual bool IsValidCfg(const double cfg[]) {
            return IsWithinLimitsCfg(cfg) && IsObstacleCollisionFreeCfg(cfg) && IsSelfCollisionFreeCfg(cfg);
        }

        // cfg path functionality
        virtual void PathCfgs(const double cfg1[], const double cfg2[], const double t, double cfg[]);

        virtual void AddToMeshPathCfgs(TriMesh &tmesh, const double cfg1[], const double cfg2[], const double tstart,
                                       const double dt, const double tend);

        virtual bool
        IsWithinLimitsPathCfgs(const double cfg1[], const double cfg2[], const double tstart, const double dt,
                               const double tend);

        virtual bool
        IsObstacleCollisionFreePathCfgs(const double cfg1[], const double cfg2[], const double tstart, const double dt,
                                        const double tend);

        virtual bool
        IsSelfCollisionFreePathCfgs(const double cfg1[], const double cfg2[], const double tstart, const double dt,
                                    const double tend);

        virtual bool IsValidPathCfgs(const double cfg1[], const double cfg2[], const double tstart, const double dt,
                                     const double tend) {

            return IsWithinLimitsPathCfgs(cfg1, cfg2, tstart, dt, tend) &&
                   IsObstacleCollisionFreePathCfgs(cfg1, cfg2, tstart, dt, tend) &&
                   IsSelfCollisionFreePathCfgs(cfg1, cfg2, tstart, dt, tend);
        }

        // distances
        virtual double DistanceStates(const double s1[], const double s2[]) const = 0;

        virtual double DistanceStatePosition(const double s[], const double pos[]) const;

        virtual double DistanceStateCfg(const double s[], const double cfg[]) const;

        virtual double DistanceCfgs(const double cfg1[], const double cfg2[]) const {
            return Algebra::PointDistance(GetCfgAllocator()->GetDim(), cfg1, cfg2);
        }

        virtual double DistancePositions(const double p1[], const double p2[]) const {
            return Algebra::PointDistance(GetPositionAllocator()->GetDim(), p1, p2);
        }

        // projections and unprojections
        virtual void GetPositionState(const double s[], double pos[]) const = 0;

        virtual void GetCfgState(const double s[], double cfg[]) const = 0;

        virtual void GetPositionCfg(const double cfg[], double pos[]) const = 0;

        virtual std::vector<std::vector<double>> GetPathDataToGoals(){
            return pathToGoalFromMakePlan;
        }

        virtual void SetPathDataToGoals(const std::vector<std::vector<double>> &p){
            pathToGoalFromMakePlan = p;
        }

        virtual void SetRobotState(const std::vector<double> &r){
            robotState = r;
        }

        virtual void SetRobot(Robot_config* r) {
            robot = r;
        }

        virtual Robot_config* GetRobot(){
            return robot;
        }

        virtual std::vector<double> GetRobotState(){
            return robotState;
        }

        virtual void SetDistanceToGoal(double l){
            los = l;
        }

        virtual std::vector<std::vector<double>> GetLaserData(){
            return laserData;
        }

        virtual double GetDistanceToGoal(){
            return los;
        }

        virtual void SetLaserData(const std::vector<std::vector<double>> &l){
            laserData = l;
        }

        virtual void SetPositionState(const double pos[], double s[]) const = 0;

        virtual void SetCfgState(const double cfg[], double s[]) const = 0;

        virtual void SetPositionCfg(const double pos[], double cfg[]) const = 0;

        // clearance from obstacles
        virtual double ClearanceState(void);

        virtual double ClearancePosition(const double pos[]);

        virtual double ClearanceSegment(const double p1[], const double p2[]);

        virtual double ClearanceCfg(const double cfg[]);

        virtual double ClearancePathCfgs(const double cfg1[], const double cfg2[], const double tstart, const double dt,
                                         const double tend);

        // other functionality
        virtual double GetVelocityState(const double s[]) const = 0;

        virtual void SamplePosition(double pos[]) {
            GetScene()->GetGrid()->SamplePoint(pos);
        }

        virtual double SimulateOneStep(void) = 0;

        virtual void SimulateOneStepFromParams(double v, double w, double dt) = 0;

        virtual void DrawState(void) = 0;

        virtual void DrawCfg(const double cfg[]) const;

        virtual void DrawPosition(const double pos[]) const = 0;

        virtual void DrawSegment(const double pos1[], const double pos2[]) const = 0;

        void SetModelPath(const char planner[], const char map[]) {
            modelPath = std::string(planner) + std::string(map);
        }

        void LoadPredictModels(void) {

        }

        double best_w;
        double best_v;

    protected:
        virtual double TimeStepConstantVelocity(const double v, const double d) const;

        virtual double TimeStepConstantAcceleration(const double v, const double a, const double d) const;

        double m_dt;
        double m_minDistanceOneStep;
        double m_maxDistanceOneStep;
        double m_distanceReachedSteerPosition;
        double m_stopDistance;
        double m_stopExponent;
        double m_velocityScaleSampling;
        double m_velocityScaleConversion;
        double m_velocityScaleMin;
        double m_drawPositionRadius;
        double *m_currState;
        double *m_currControl;
        std::string modelPath;
        Allocator m_stateAllocator;
        Allocator m_cfgAllocator;
        Allocator m_positionAllocator;
        Allocator m_controlAllocator;

        std::vector<std::vector<double>> pathToGoalFromMakePlan;
        std::vector<std::vector<double>> laserData;
        std::vector<double> robotState;

        Robot_config* robot;
        double los;
    };

    ClassContainer(Simulator, m_simulator);
}

#endif
