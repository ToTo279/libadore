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

#include <adore/env/map/occupancy_grid.h>
#include <eigen3/Eigen/Core>
#include <adore/fun/node.h>
#include <adore/fun/search_grid.h>
#include <adore/fun/hybrid_A_star.h>
#include <plotlablib/figurestubfactory.h>
#include <adore/fun/collision_check_offline.h>
//#include <adore/fun/vornoi_diagram.h>//
#include <plotlablib/afigurestub.h>
#include <geometry_msgs/Pose.h>
//#include <adore/mad/catmull_rom_splines.h>//
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <adore/fun/trajectory_smoothing.h>
#include <ctime>
#include <chrono>


//New:
//#include "Vector3.h"
//#include "Quaternion.h"
//#include <Matrix3x3.h>

#include <adore_if_ros/graph_search_scheduling.h>

namespace adore
{
namespace apps
{
    /**
     * 
     */
    class GraphSearch
    {
        private:
        //typedef boost::geometry::model::point<double,2,boost::geometry::cs::cartesian> Vector;
            DLR_TS::PlotLab::FigureStubFactory fig_factory;
            DLR_TS::PlotLab::AFigureStub* figure3;  
            DLR_TS::PlotLab::AFigureStub* figure4; 
            DLR_TS::PlotLab::AFigureStub* figure5;         
            int time1, time2;
            std::chrono::system_clock::time_point  start;
            std::chrono::system_clock::time_point  end;        
            static const int Length = 80; //73;
            static const int Width = 20;//20;  
            static const int HeadingResolution = 5;  
            static const int nH_Type = 3;  //non holonomic
            static const int H_Type = 2;  //holonomic
            int Depth;      
            adore::env::OccupanyGrid OG;
            adore::fun::GRID<adore::fun::Node<nH_Type,double>> NH_GRID;
            ros::Subscriber StartPose_subscreiber; 
            ros::Subscriber EndPose_subscreiber; 
            //adore::fun::ArrayFormGrid<adore::fun::Node<nH_Type,double>> NH_GRID;
            adore::fun::Hybrid_A_Star* h_A_star;
            adore::fun::Node<3,double> Start;
            adore::fun::Node<3,double> End;
            bool validStart, validEnd;
            adore::fun::CollisionCheckOffline* cco;
            ros::NodeHandle* node_;
            fun::TrajectorySmoothing* smoothing;
            double avg_time;
            double vehicleLength, vehicleWidth;
            int iteration;

        
        public:
        GraphSearch(ros::NodeHandle* parentnode)
        {
            vehicleLength = 3.2;
            vehicleWidth = 1.0; 
            smoothing = new fun::TrajectorySmoothing;
            h_A_star = new adore::fun::Hybrid_A_Star(smoothing);
            node_ = parentnode;
            figure3 = fig_factory.createFigureStub(3);
            figure3->showAxis();
            figure3->showGrid();
            figure3->show();  
            figure4 = fig_factory.createFigureStub(4);
            figure4->showAxis();
            figure4->showGrid();
            figure4->show();   
            figure5 = fig_factory.createFigureStub(5);
            figure5->showAxis();
            figure5->showGrid();
            figure5->show();                               
            Depth = 360 / HeadingResolution;
            cco = new adore::fun::CollisionCheckOffline(vehicleWidth, vehicleLength, HeadingResolution, 10);
            OG.resize(Width,Length,figure3);
            NH_GRID.resize(Width,Length,Depth);
            h_A_star->setSize(Width,Length);
            avg_time = 0.0;
            iteration = 1;
            


        }
        void setConstPenalty(double value)
        {
            const_penalty_ = value;
        }
        void setSpeedScale(double value)
        {
            nominal_planner_->setSpeedScale(value);
        }

        void setStopPoint(int value)
        {
        if(value<0)
            {
            nominal_planner_->setConflictSet(nullptr);
            }
            else
            {
            nominal_planner_->setConflictSet(&conflicts_);
            }

        }

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

            three_lanes_.update();
            auto current = three_lanes_.getCurrentLane();
            ngo_.update();
            connectionSet_.update();
            connectionsOnLane_->update();
            checkPointSet_.update();
            checkPointsOnLane_->update();
            conflicts_.update();


            if(!current->isValid())
            {
                planning_result.status_string = "current lane invalid";
                return;
            }

            prediction_.update();

            auto x0=planning_request.initial_state.toMotionState();
            nominal_planner_->compute(x0);
            if(!nominal_planner_->hasValidPlan())
            {
            planning_result.status_string = "nominal maneuver planning failed, "+ nominal_planner_->getStatus();
            return;
            }

            nominal_planner_->getSetPointRequest()->copyTo(planning_result.nominal_maneuver);
            planning_result.nominal_maneuver_valid = true;
            auto x0em = nominal_planner_->getSetPointRequest()->interpolateSetPoint(planning_request.t_emergency_start,pvehicle_);
            //emergency_planner_->setJMax(-pEmergencyOperation_->getEmergencyManeuverJMin());
            //emergency_planner_->setTStall(pEmergencyOperation_->getEmergencyManeuverTStall());

            // adore::fun::RestartEffort re;  //test whether trajectory is long enough to justify restarting from ~0
            // if(!re.isValid(planning_result.nominal_maneuver))
            // {
            //   planning_result.status_string = "not restarting, maneuver too short";
            //   return;
            // }

            //if emergencyManeuverAMax and AMin are different, compute three maneuvers for {amin, (amin+amax)/2, amax}
            //otherwise compute just one maneuver for amin
            /*std::vector<double> emergency_acceleration;
            emergency_acceleration.push_back(pEmergencyOperation_->getEmergencyManeuverAMin());
            if(  pEmergencyOperation_->getEmergencyManeuverAMax()
            - pEmergencyOperation_->getEmergencyManeuverAMin() > 0.01 )
            {
            emergency_acceleration.push_back((pEmergencyOperation_->getEmergencyManeuverAMin()
                                            + pEmergencyOperation_->getEmergencyManeuverAMax())*0.5);
            emergency_acceleration.push_back(pEmergencyOperation_->getEmergencyManeuverAMax());
            }

            bool collision_detection_passed = false;

            for(double a_em:emergency_acceleration)
            {
            emergency_planner_->setAStall(a_em);
            emergency_planner_->setAMin(a_em);
            emergency_planner_->compute(x0em.toMotionState());

            if(!emergency_planner_->hasValidPlan() ) 
            {
                continue;
            }*/

            planning_result.combined_maneuver.setPoints.clear();
            nominal_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,0);
            planning_result.combined_maneuver.removeAfter(planning_request.t_emergency_start);
            planning_result.combined_maneuver.setPoints.back().tEnd = planning_request.t_emergency_start;
            //emergency_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,1);


            if(collision_detection_.isValid(planning_result.combined_maneuver))
            {
                collision_detection_passed = true;
                break;
            }
            //}


            /*if(!emergency_planner_->hasValidPlan() ) 
            {
            planning_result.status_string += "emergency maneuver planning failed";
            return;
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
            adore::fun::SPRNavigationCostOnLane navcostOnLane(&three_lanes_,pTacticalPlanner_,laneid);
            planning_result.objective_values.insert({navcostOnLane.getName(),
                                                    navcostOnLane.getCost(planning_result.nominal_maneuver)});

            adore::fun::SPRNormalizedNavigationCost navcostNormalized(&three_lanes_,pTacticalPlanner_,laneid);
            planning_result.objective_values.insert({navcostNormalized.getName(),
                                                    navcostNormalized.getCost(planning_result.nominal_maneuver)});

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
    }
}
}

    