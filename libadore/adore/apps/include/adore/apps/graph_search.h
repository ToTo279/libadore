/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *    Reza Dariani- initial implementation and API
 ********************************************************************************/
#pragma once

#include <adore/fun/afactory.h>
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/fun/tac/basiclanefollowingplanner.h>
#include <adore/fun/tac/basicmrmplanner.h>
#include <adore/env/navigationgoalobserver.h>
#include <adore/env/tcd/connectionsonlane.h>
//#include <adore/env/threelaneviewdecoupled.h>
#include <adore/fun/tac/basicsetpointrequestevaluators.h>
#include <adore/env/traffic/decoupledtrafficpredictionview.h>
#include <adore/apps/trajectory_planner_base.h>
#include <adore/fun/safety/setpointrequestswath.h>
#include <adore/env/traffic/decoupledconflictpointview.h>


//New:
//#include "Vector3.h"
//#include "Quaternion.h"
//#include <Matrix3x3.h>

#include <adore/fun/basicunstructuredplanner.h>

namespace adore
{
  namespace apps
  {
    /**
     * @brief Decoupled trajectory planner, which uses TrajectoryPlannerBase to compute and provide a PlanningResult in the event of a PlanningRequest
     */
    
    class GraphSearch:public TrajectoryPlannerBase
    {
      private:
      typedef adore::fun::BasicUnstructuredPlanner TUnstructuredPlanner;
      TUnstructuredPlanner* basicunstructuredplanner_;


      adore::params::APVehicle* pvehicle_;
      adore::params::APTacticalPlanner* pTacticalPlanner_;
      adore::params::APTrajectoryGeneration* pTrajectoryGeneration_;
      adore::params::APPrediction* ppred_;
      //adore::env::NavigationGoalObserver ngo_;
      adore::env::ControlledConnectionSet4Ego connectionSet_;/**< state of controlled connections in area*/
      adore::env::ControlledConnectionSet4Ego checkPointSet_;/**< state of checkPoints in area*/
      adore::env::ConnectionsOnLane* connectionsOnLane_;/** map controlled connections to lane*/
      adore::env::ConnectionsOnLane* checkPointsOnLane_;/** map controlled connections to lane*/
      adore::env::DecoupledTrafficPredictionView prediction_;/**<collision detection based representation of traffic*/
      adore::fun::SPRTTCNominal ttcCost_;/**<collision detection based ttc computation*/
      adore::fun::SPRNonCoercive coercion_detection_;/**<collision detection vs expected behavior*/
      //adore::env::DecoupledConflictPointView conflicts_;/**cross traffic conflicts*/

      /**
       * combined maneuver post-processing constraints
       */
      adore::fun::SPRInvariantCollisionFreedom collision_detection_;/**<collision detection with traffic predictions*/


      int id_;/**<integral id to be written to PlanningResult*/
      std::string plannerName_;/**human readable planner name written to PlanningResult*/
      double lateral_i_grid_;/**grid index*/
      double const_penalty_;/**penalty, which is always added to cost*/
      public:
      virtual ~GraphSearch()
      {
            delete basicunstructuredplanner_;
      }

      GraphSearch(int id=0,std::string plannerName = "graph_search",double lateral_i_grid = 0.0):
           connectionSet_(adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader(),
                          adore::env::EnvFactoryInstance::get()->getControlledConnectionFeed()),
           checkPointSet_(adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader(),
                          adore::env::EnvFactoryInstance::get()->getCheckPointFeed()),
           //ngo_(adore::env::EnvFactoryInstance::get(),three_lanes_.getCurrentLane(),0,0),
           prediction_(),
           coercion_detection_(&prediction_),
           //conflicts_(three_lanes_.getCurrentLane()),
           collision_detection_(&prediction_),
           ttcCost_(&prediction_)
      {
        id_ = id;
        const_penalty_ = 0.0;
        plannerName_ = plannerName;
        pvehicle_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
        pTacticalPlanner_ = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        pTrajectoryGeneration_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
        ppred_ = adore::params::ParamsFactoryInstance::get()->getPrediction();
        auto pTacticalPlanner = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        //connectionsOnLane_ = new adore::env::ConnectionsOnLane(three_lanes_.getCurrentLane(),&connectionSet_);
        //checkPointsOnLane_ = new adore::env::ConnectionsOnLane(three_lanes_.getCurrentLane(),&checkPointSet_);
        //create nominal planner and add additional constraints

      }





        /*void setConstPenalty(double value)
        {
            const_penalty_ = value;
        }
        void setSpeedScale(double value)
        {
            basicunstructeredplanner_->setSpeedScale(value);
        }

        void setStopPoint(int value)
        {
        if(value<0)
            {
            basicunstructeredplanner_->setConflictSet(nullptr);
            }
            else
            {
            basicunstructeredplanner_->setConflictSet(&conflicts_);
            }

        }*/

        /**
        * @brief update data, views and recompute maneuver
        * 
        */
        virtual void computeTrajectory(const adore::fun::PlanningRequest& planning_request, adore::fun::PlanningResult& planning_result) override
        {
            //document planner result
            planning_result.id = id_; // in lfbehavior, there is only one maneuver
            planning_result.name = plannerName_;
            planning_result.maneuver_type = adore::fun::PlanningResult::NOMINAL_DRIVING;
            planning_result.iteration = planning_request.iteration;
            planning_result.nominal_maneuver_valid = false;
            planning_result.combined_maneuver_valid = false;

            //three_lanes_.update();
            //auto current = three_lanes_.getCurrentLane();
            //ngo_.update();
            connectionSet_.update();
            connectionsOnLane_->update();
            checkPointSet_.update();
            checkPointsOnLane_->update();
            //conflicts_.update();


            /*if(!current->isValid())
            {
                planning_result.status_string = "current lane invalid";
                return;
            }*/

            prediction_.update();

            auto x0=planning_request.initial_state.toMotionState();


            basicunstructuredplanner_->compute(x0);
            if(!basicunstructuredplanner_->hasValidPlan())
            {
                planning_result.status_string = "nominal maneuver planning failed, ";//+ basicunstructuredplanner_->getStatus();
                return;
            }


            
            basicunstructuredplanner_->getSetPointRequest()->copyTo(planning_result.nominal_maneuver);
            planning_result.nominal_maneuver_valid = true;
            //auto x0em = basicunstructeredplanner_->getSetPointRequest()->interpolateSetPoint(planning_request.t_emergency_start,pvehicle_);

            bool collision_detection_passed = false;

            planning_result.combined_maneuver.setPoints.clear();
            basicunstructuredplanner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,0);
            planning_result.combined_maneuver.removeAfter(planning_request.t_emergency_start);
            planning_result.combined_maneuver.setPoints.back().tEnd = planning_request.t_emergency_start;

            /*if(collision_detection_.isValid(planning_result.combined_maneuver))
            {
                collision_detection_passed = true;
                break;
            }*/


            double front_buffer_space = pTacticalPlanner_->getCollisionDetectionFrontBufferSpace();
            double lateral_precision = pTacticalPlanner_->getCollisionDetectionLateralPrecision();
            adore::fun::SetPointRequestSwath spr_swath(
                pvehicle_->get_a()+pvehicle_->get_b()+pvehicle_->get_c()+pvehicle_->get_d()+front_buffer_space,
                pvehicle_->get_bodyWidth(),
                pvehicle_->get_d(),//spr reference point at rear axle
                lateral_precision);
            spr_swath.setLonError(pTacticalPlanner_->getCollisionDetectionLongitudinalError());
            spr_swath.setLatError(pTacticalPlanner_->getCollisionDetectionLateralError());
            spr_swath.setDuration(ppred_->get_roadbased_prediction_duration());
            spr_swath.append_cylinder_swath_linear(planning_result.combined_maneuver,planning_result.combined_maneuver_swath,true);
            spr_swath.setAccelerationErrorSlow(pTacticalPlanner_->getNominalSwathAccelerationError());
            spr_swath.append_cylinder_swath_linear(planning_result.nominal_maneuver,planning_result.nominal_maneuver_swath,true);

            if(!collision_detection_passed)
            {
            planning_result.status_string = "collision detected for combined maneuver";    
            return;          
            }

            planning_result.combined_maneuver.cropAfterFirstStop(0.3);
            if(planning_result.combined_maneuver.setPoints.size()==0)
            {
            planning_result.status_string = "stopping";
            return;
            }

            planning_result.combined_maneuver_valid = true;
            int laneid = 0;
            /*adore::fun::SPRNavigationCostOnLane navcostOnLane(&three_lanes_,pTacticalPlanner_,laneid);
            planning_result.objective_values.insert({navcostOnLane.getName(),
                                                    navcostOnLane.getCost(planning_result.nominal_maneuver)});

            adore::fun::SPRNormalizedNavigationCost navcostNormalized(&three_lanes_,pTacticalPlanner_,laneid);
            planning_result.objective_values.insert({navcostNormalized.getName(),
                                                    navcostNormalized.getCost(planning_result.nominal_maneuver)});
            */

            adore::fun::SPRLongitudinalAcceleration2Cost lacccost;
            planning_result.objective_values.insert({lacccost.getName(),
                                                    lacccost.getCost(planning_result.nominal_maneuver)});

            adore::fun::SPRLongitudinalJerk2Cost ljerkcost;
            planning_result.objective_values.insert({ljerkcost.getName(),
                                                    ljerkcost.getCost(planning_result.nominal_maneuver)});

            adore::fun::SPRAverageProgressLoss progressLoss(pTacticalPlanner_);
            planning_result.objective_values.insert({progressLoss.getName(),
                                                    progressLoss.getCost(planning_result.nominal_maneuver)});

            planning_result.objective_values.insert({"const_penalty",const_penalty_});

            //ttc cost
            planning_result.objective_values.insert({ttcCost_.getName(),
                                                    ttcCost_.getCost(planning_result.nominal_maneuver)});


            switch(pTacticalPlanner_->getCoercionPreventionStrategy())
            {
            case 0://turned off
                {
                planning_result.objective_values.insert({coercion_detection_.getName(),0.0});
                }
                break;
            case 1://encode as objective value
                {
                planning_result.objective_values.insert({coercion_detection_.getName(),
                                                            coercion_detection_.isValid(planning_result.nominal_maneuver)
                                                            ? 0.0 : 1.0});
                }
                break;
            case 2://encode as constraint
                {
                bool coercion_detection_passed = coercion_detection_.isValid(planning_result.nominal_maneuver);
                planning_result.combined_maneuver_valid = coercion_detection_passed && planning_result.combined_maneuver_valid;
                if(!coercion_detection_passed)
                {
                    planning_result.status_string = "coercion detected for nominal maneuver";    
                }
                }
                break;
            }
        }
    };
}
}

    