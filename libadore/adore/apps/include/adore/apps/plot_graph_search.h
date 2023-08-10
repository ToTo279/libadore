#pragma once
#include <plotlablib/afigurestub.h>
#include <plotlablib/figurestubfactory.h>
#include <adore/apps/if_plotlab/plot_shape.h>
#include <boost/container/vector.hpp>

#include <boost/geometry.hpp>
#include <eigen3/Eigen/Dense>

#include <plotlablib/figurestubfactory.h>
#include <plotlablib/afigurestub.h>

#include <boost/container/vector.hpp>
#include <adore/fun/tree_builder.h>
#include <plotlablib/figurestubfactory.h>
#include <plotlablib/afigurestub.h>
#include <adore/mad/cubicpiecewisefunction.h>
#include <adore/mad/coordinateconversion.h>
#include "csaps.h"

//#include <adore/fun/node.h>
//#include <adore/env/map/occupancy_grid.h>
//#include <adore/fun/dubins_curve.h>

namespace adore
{
  namespace apps
  {

    namespace bg = boost::geometry; 
    namespace bgm = bg::model;
    using point_xy = bgm::d2::point_xy<double>; 
    using polygon = bgm::polygon<point_xy>;

    class PlotGraphSearch
    {
        private:
            //typedef adore::env::OccupanyGrid TOccupanyGrid;
            //TOccupanyGrid* occupany_grid;
            adore::env::OccupanyGrid occupany_grid;
            
            DLR_TS::PlotLab::FigureStubFactory fig_factory;
            double pi;
            std::string GREEN= "LineColor=0.75,1.,0.75;LineWidth=2";
            std::string RED= "LineColor=0.,0.,0.;LineWidth=3";
        
        public:
            DLR_TS::PlotLab::AFigureStub* figure3;  
            DLR_TS::PlotLab::AFigureStub* figure4; 
            DLR_TS::PlotLab::AFigureStub* figure5;

            typedef bg::model::point<double,2,bg::cs::cartesian> Point;
            typedef bg::model::box<Point> box;
            /*polygon rectangularBox;
            struct circle
            {
                double x, y, r;
            };            
            struct _Obstacle
            {
                double x;
                double y;
                double width;
                double length;
                double alpha;
                std::vector<double> vertices_x,vertices_y;
                std::vector<circle> circles;
                polygon poly;
                int ID;
            };*/

            //Eigen::MatrixXd Grid; 
            //typedef boost::container::vector<_Obstacle> obstacleList;
            //obstacleList obstacles;
            
            




            PlotGraphSearch()
            {
              pi = 3.141592653589793;
            }

            ~PlotGraphSearch()
            {

            }

            void createFigure()
            {
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
            }



            void plotSoftRectangle(adore::env::OccupanyGrid::_Obstacle* obst,DLR_TS::PlotLab::AFigureStub* figure,std::string tag)
            {
                std::vector<double> x_v, y_v;
                double x,y,xt,yt ;
                for (double beta = 0.0; beta < 2*pi ; beta = beta + 0.1745)
                {
                    occupany_grid.polar2Cartesian(x, y, occupany_grid.get_softRectangle_r (beta, obst->length, obst->width), beta);
                    occupany_grid.transformation(xt, yt,obst->alpha, x, y);
                    x_v.push_back(obst->x + xt);
                    y_v.push_back(obst->y + yt);
                    //std::cout<<"\n"<<x<<"\t"<<y;
                }
                figure->plot(tag,&x_v[0],&y_v[0],2.5,x_v.size(), GREEN);
            }

            void plotEllipse(adore::env::OccupanyGrid::_Obstacle* obst,DLR_TS::PlotLab::AFigureStub* figure,std::string tag)
            {
                std::vector<double> x_v, y_v;
                double x,y,xt,yt ;
                for (double beta = 0.0; beta < 2*pi ; beta = beta + 0.1745)
                {
                    occupany_grid.polar2Cartesian(x, y, occupany_grid.get_ellipse_r (beta, obst->length, obst->width), beta);
                    occupany_grid.transformation(xt, yt,obst->alpha, x, y);
                    x_v.push_back(obst->x + xt);
                    y_v.push_back(obst->y + yt);
                    //std::cout<<"\n"<<x<<"\t"<<y;
                }
                figure->plot(tag,&x_v[0],&y_v[0],2.5,x_v.size(), GREEN);
            }            
            void plotPolygon(adore::env::OccupanyGrid::_Obstacle* obst,DLR_TS::PlotLab::AFigureStub* figure,std::string tag)
            {
               //std::cout<<"\n****** "<<obst->vertices_x.size();
                figure->plot(tag,&obst->vertices_x[0],&obst->vertices_y[0],2.5,obst->vertices_x.size(), GREEN);

            }
            void PLOT(DLR_TS::PlotLab::AFigureStub* figure)
            {
                
                              
                std::stringstream ss;
                for (int r=0; r<occupany_grid.Grid.rows(); ++r)
                {                    
                    for(int c=0; c<occupany_grid.Grid.cols(); ++c)
                    {
                        ss.clear();
                        ss.str("");
                        ss << "f"<<r*occupany_grid.Grid.cols()+c;
                        if(occupany_grid.Grid(r,c)) PLOT::plotPosition(ss.str(),c,r,figure,RED,0.05);
                        //std::cout<<"\n"<<r<<"\t"<<c<<"\t"<<r*Grid.cols()+c;
                        else PLOT::plotPosition(ss.str(),c,r,figure,GREEN,0.05);
                    }

                }
                
            }

            /*void plot_dubin(DLR_TS::PlotLab::AFigureStub* figure)
            {
                int size = adore::fun::DubinsCurve::path [optIndex].curve.size();
                std::vector<double> x, y, psi;
                for (int i=0; i< size; ++i)
                {
                    std::stringstream ss;
                    ss.str("");
                    ss << "fff"<<i*2;  
                    PLOT::plotRectangle(ss.str(), path [optIndex].curve[i].x, path [optIndex].curve[i].y, 2.0, 2.0, figure, GRAY, path [optIndex].curve[i].psi);                
                    x.push_back(path [optIndex].curve[i].x);
                    y.push_back(path [optIndex].curve[i].y);
                    psi.push_back(path [optIndex].curve[i].psi);
                }
                figure->plot("#d_c",x.data(),y.data(), 1.2, size, color[1]);                   
            }*/
    };

  }
}