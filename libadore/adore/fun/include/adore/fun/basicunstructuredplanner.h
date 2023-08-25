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

//NEW
//#include <adore/env/map/occupancy_grid.h>
//#include <adore/apps/plot_graph_search.h>

#include <adore/env/map/occupancy_grid.h>
#include <eigen3/Eigen/Core>
#include <adore/fun/node.h>
#include <adore/fun/search_grid.h>
#include <adore/fun/hybrid_A_star.h>
//#include <plotlablib/figurestubfactory.h>
#include <adore/fun/collision_check_offline.h>
#include <adore/fun/tac/anominalplanner.h>
//#include <libadore/libadore/adore/fun/include/adore/fun/tac/anominalplanner.h>
//#include <adore/fun/vornoi_diagram.h>//
//#include <plotlablib/afigurestub.h>
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
#include <adore/fun/setpointrequest.h>



//New:
//#include "Vector3.h"
//#include "Quaternion.h"
//#include <Matrix3x3.h>
//#include <adore/fun/dubins_curve.h>
#include <adore/apps/plot_occupancy_grid.h>


namespace adore
{
//namespace apps
//{
    /**
     * 
     */
namespace fun
{
    class BasicUnstructuredPlanner : public ANominalPlanner
    {
        private:
            typedef adore::apps::PlotOccupanyGrid TPlotOccupanyGrid;
            TPlotOccupanyGrid* plot_;

            std::chrono::system_clock::time_point  startTime;
            std::chrono::system_clock::time_point  endTime;        
            static const int Length = 80; //73;
            static const int Width = 20;//20;  
            static const int HeadingResolution = 5;  
            static const int nH_Type = 3;  //non holonomic
            static const int H_Type = 2;  //holonomic
            int Depth;      
            adore::env::OccupanyGrid OG;
            adore::fun::GRID<adore::fun::Node<nH_Type,double>> NH_GRID;
            //adore::fun::ArrayFormGrid<adore::fun::Node<nH_Type,double>> NH_GRID;
            adore::fun::Hybrid_A_Star* h_A_star;
            adore::fun::Node<3,double> Start;
            adore::fun::Node<3,double> End;
            bool validStart, validEnd;
            adore::fun::CollisionCheckOffline* cco;
            fun::TrajectorySmoothing* smoothing;
            double avg_time;
            double vehicleLength, vehicleWidth;
            int iteration;
            bool hasValidPlan_;
            SetPointRequest spr_;

        
        public:
        BasicUnstructuredPlanner()
        {
            vehicleLength = 3.2;
            vehicleWidth = 1.0; 
            smoothing = new fun::TrajectorySmoothing;
            h_A_star = new adore::fun::Hybrid_A_Star(smoothing);
            

            plot_->initialize_plot();

            Depth = 360 / HeadingResolution;
            cco = new adore::fun::CollisionCheckOffline(vehicleWidth, vehicleLength, HeadingResolution, 10);
            plot_->resize(Width,Length,plot_->figure3);
            NH_GRID.resize(Width,Length,Depth);
            h_A_star->setSize(Width,Length);
            avg_time = 0.0;
            iteration = 1;
            hasValidPlan_ = false;
        }
        
        virtual void compute(const VehicleMotionState9d& initial_state) override
        {
            hasValidPlan_ = false;

            setStart(initial_state);
            //TO DO: final_state erstellen
            VehicleMotionState9d final_state;
            
            final_state.setX(initial_state.getX()+20);
			final_state.setY(initial_state.getY()+20);
			final_state.setZ(initial_state.getZ());
			final_state.setPSI(initial_state.getPSI());
			final_state.setvx(initial_state.getvx());
			final_state.setvy(initial_state.getvy());
			final_state.setOmega(initial_state.getOmega());
			final_state.setAx(initial_state.getAx());
			final_state.setDelta(initial_state.getDelta());

            setEnd(final_state);

            while(iteration<2 && validStart && validEnd)
            {

                std::cout<<"\nITERATION: "<<iteration;
                startTime = std::chrono::system_clock::now();
                //time1 = 0.0; error: ‘time1’ was not declared in this scope; did you mean ‘time’?
                //time2 = 0.0;
                   
                //std::cout<<"\n"<<   cco->offlineCollisionTable.size()<<"\t"<<cco->offlineCollisionTable[0].size1()<<"\t"<<cco->offlineCollisionTable[0].size2();   
                hasValidPlan_ = h_A_star->plan(&NH_GRID,&OG, cco, &Start,&End,HeadingResolution,1000, vehicleWidth, vehicleLength ,plot_->figure3,plot_->figure4,plot_->figure5);            
                endTime = std::chrono::system_clock::now(); 
                iteration++;          
                
            }
        }
        /*
                    adore::fun::SetPoint sp;
                    sp.tStart = j*dt;
                    sp.tEnd = sp.tStart + dt;
                    sp.x0ref.setX(states[X].at(j));
                    sp.x0ref.setY(states[Y].at(j));
                    sp.x0ref.setPSI(states[PSI].at(j));
                    sp.x0ref.setvx((j==0 ? (states[S].at(j+1)-states[S].at(j)) : (states[S].at(j)-states[S].at(j-1)))/dt);
                    sp.x0ref.setvy(0);
                    sp.x0ref.setOmega((j==0 ? (states[PSI].at(j+1)-states[PSI].at(j)) : (states[PSI].at(j)-states[PSI].at(j-1)))/dt);
                    if(j>1)
                    {
                        sp.x0ref.setAx((sp.x0ref.getvx() - spr.setPoints.back().x0ref.getvx())/dt);
                    }
                    else if(j==1)
                    {
                        sp.x0ref.setAx((sp.x0ref.getvx() - spr.setPoints.back().x0ref.getvx())/dt);
                        spr.setPoints.back().x0ref.setAx((sp.x0ref.getvx() - spr.setPoints.back().x0ref.getvx())/dt);
                    }
                    sp.x0ref.setDelta(control.at(j));
                    if(j>1)
                    {
                        sp.x0ref.setDAx((sp.x0ref.getAx() - spr.setPoints.back().x0ref.getAx())/dt);
                    }
                    else if(j==1)
                    {
                        sp.x0ref.setDAx((sp.x0ref.getAx() - spr.setPoints.back().x0ref.getAx())/dt);
                        spr.setPoints.back().x0ref.setDAx((sp.x0ref.getAx() - spr.setPoints.back().x0ref.getAx())/dt);
                    }
                    sp.x0ref.setDDelta((j==0 ? (control.at(j+1)-control.at(j)) : (control.at(j)-control.at(j-1)))/dt);

                    spr.push_back(sp);
                    
        */

 
        virtual bool hasValidPlan()const override
        {
            return hasValidPlan_;
        }
        /**
         * getSetPointRequest - return computed trajectory in the form of a SetPointRequest
         */
        virtual const SetPointRequest* getSetPointRequest()const override
        {
            return h_A_star->getSetPointRequest();
        }
        /**
         *  getCPUTime - return the time require for trajectory planning in seconds
         */
        virtual double getCPUTime()const  override
        {
            return std::chrono::duration<double>(endTime - startTime).count(); 
        }




    void setStart(const VehicleMotionState9d& initial_state)
    {
        validStart = Start.setPosition(initial_state.getX(),initial_state.getY(),initial_state.getPSI(),Width,Length,Depth,adore::mad::CoordinateConversion::DegToRad(HeadingResolution), plot_->figure3);
        //Start.print();
    }  
    void setEnd(const VehicleMotionState9d& final_state)
    {
        validEnd = End.setPosition(final_state.getX(),final_state.getY(),final_state.getPSI(),Width,Length,Depth, adore::mad::CoordinateConversion::DegToRad(HeadingResolution),  plot_->figure3);
        //End.print();
    }                
    void getOccupancies_x(std::vector<double>& v)
    {
        OG.getOccupancies_x(v);
    }

    void getOccupancies_y(std::vector<double>& v)
    {
        OG.getOccupancies_y(v);
    }

    int getWidth()
    {
        return OG.getWidth();
    }
    int getLength()
    {
        return OG.getLength();
    }
             
    };
}
}