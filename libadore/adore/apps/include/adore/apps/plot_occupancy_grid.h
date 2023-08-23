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



namespace adore
{
  namespace apps
  {

    namespace bg = boost::geometry; 
    namespace bgm = bg::model;
    using point_xy = bgm::d2::point_xy<double>; 
    using polygon = bgm::polygon<point_xy>;

    class PlotOccupanyGrid
    {
        private:
            adore::env::OccupanyGrid occupany_grid;
            
            double pi;
            std::string GREEN= "LineColor=0.75,1.,0.75;LineWidth=2";
            std::string RED= "LineColor=0.,0.,0.;LineWidth=3";

            DLR_TS::PlotLab::FigureStubFactory* fig_factory;

            adore::mad::AFeed<adore::fun::PlanningResult>* planning_result_feed_;

            Eigen::MatrixXd Grid;
            //typedef boost::container::vector<_Obstacle> obstacleList;
            //obstacleList obstacles;
            //polygon rectangularBox;
        
        public:
            DLR_TS::PlotLab::AFigureStub* figure3;  
            DLR_TS::PlotLab::AFigureStub* figure4; 
            DLR_TS::PlotLab::AFigureStub* figure5;

            typedef bg::model::point<double,2,bg::cs::cartesian> Point;
            typedef bg::model::box<Point> box;

            std::vector<int> occupancies_x;
            std::vector<int> occupancies_y;

            void initialize_plot()
            {
                fig_factory = new DLR_TS::PlotLab::FigureStubFactory();
                figure3 = fig_factory->createFigureStub(3);
                figure3->showAxis();
                figure3->showGrid();
                figure3->show();  
                figure4 = fig_factory->createFigureStub(4);
                figure4->showAxis();
                figure4->showGrid();
                figure4->show();   
                figure5 = fig_factory->createFigureStub(5);
                figure5->showAxis();
                figure5->showGrid();
                figure5->show();
            }
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
            
            




            PlotOccupanyGrid()
            {
              pi = 3.141592653589793;
            }

            ~PlotOccupanyGrid()
            {

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

            void calculateCircles(adore::env::OccupanyGrid::_Obstacle* obst,DLR_TS::PlotLab::AFigureStub* figure)
            {
                obst->circles.clear();
                double r = std::min(obst->length, obst->width) ;
                double l = std::max(obst->length, obst->width);
                double x,y, tmp_x, tmp_y;
                std::stringstream tag;
                adore::env::OccupanyGrid::circle tmp_circle;
                for(double  i=0.0; i<l - r/4.0; i=i+r*0.5)
                {
                    tmp_x = -l/2.0 + i + r/4.0;
                    tmp_y = 0.0;;
                    adore::env::OccupanyGrid::transformation(x,y,obst->alpha, tmp_x , tmp_y);
                    x += obst->x ;
                    y += obst->y ;
                    tag << "circle"<<obst->ID<<"-"<<i;
                    tmp_circle.x = x;
                    tmp_circle.y = y;
                    tmp_circle.r = r/2.;
                    obst->circles.push_back(tmp_circle);
                    PLOT::plotCircle(tag.str(),x,y,0.10, r/2., figure,"LineColor=0.8,0.8,0.8;LineWidth=0;FillColor=1,0,0");

                }
                std::cout<<"\nNum of circles: "<<obst->circles.size();


            }

            polygon rotation(adore::env::OccupanyGrid::_Obstacle* obst,DLR_TS::PlotLab::AFigureStub* figure)
            {
                obst->vertices_x.clear();
                obst->vertices_y.clear();
                double l= obst->length;
                double w= obst->width;
                double angle = obst->alpha;
                box rectangle(Point(-l/2.0,-w/2.0),Point(l/2.0,w/2.0));
                bg::assign(occupany_grid.rectangularBox,rectangle);                
                polygon rbb;
                double x = obst->x;
                double y = obst->y;
                std::vector<Point> tmp_p;
                for(const auto&p: occupany_grid.rectangularBox.outer())
                {
                    tmp_p.push_back(Point(p.x()*cos(angle)-p.y()*sin(angle)+x,p.x()*sin(angle)+p.y()*cos(angle)+y));
                    obst->vertices_x.push_back(p.x()*cos(angle)-p.y()*sin(angle)+x);   
                    obst->vertices_y.push_back(p.x()*sin(angle)+p.y()*cos(angle)+y);                    
                }
                bg::assign_points(rbb,tmp_p);
                bg::correct(rbb);
                //std::cout<<"\n"<<bg::dsv(rbb);
                figure->plot("#obst54654",&obst->vertices_x[0],&obst->vertices_y[0],2.5,obst->vertices_x.size(), GREEN);
                
                //std::cout<<"\n"<<bg::dsv(configurationSpaceBoxes(i,j));
                return rbb;

            }

            void resize(int Width, int Length,DLR_TS::PlotLab::AFigureStub* figure =nullptr)
            {
                figure->clear();  
                Grid = Eigen::MatrixXd::Zero(Width,Length);// Width=rows & Length=colums
                adore::env::OccupanyGrid::_Obstacle obst_0;
                obst_0.ID = 0;
                obst_0.x = 16.;
                obst_0.y = 10.;
                obst_0.length = 10.;
                obst_0.width = 3.;
                obst_0.alpha = 0.17;
                obst_0.poly = rotation(&obst_0,figure); 
                occupany_grid.obstacles.push_back(obst_0);
                adore::env::OccupanyGrid::_Obstacle obst_1;
                obst_1.ID = 1;
                obst_1.x = 25.;
                obst_1.y = 2.;
                obst_1.length = 3.;
                obst_1.width = 2.;
                obst_1.alpha = 0.37;
                obst_1.poly = rotation(&obst_1,figure); 
                occupany_grid.obstacles.push_back(obst_1);                


                occupany_grid.obstacle();


                              
            }

            
            void makeGrid(DLR_TS::PlotLab::AFigureStub* figure)
            {
                int Width = occupancies_y.at(0);
                int Length = occupancies_x.at(0);
                Grid = Eigen::MatrixXd::Zero(Width,Length);

                for (int r=0; r<occupancies_y.size(); ++r)
                {                    
                    for(int c=0; c<occupancies_x.size(); ++c)
                    {
                        Grid(occupancies_y.at(r),occupancies_x.at(c)) = 1;
                    }

                }
                
            }
            void PLOT(DLR_TS::PlotLab::AFigureStub* figure)
            {
                makeGrid(figure);
                              
                std::stringstream ss;
                for (int r=0; r<Grid.rows(); ++r)
                {                    
                    for(int c=0; c<Grid.cols(); ++c)
                    {
                        ss.clear();
                        ss.str("");
                        ss << "f"<<r*Grid.cols()+c;
                        if(Grid(r,c)) PLOT::plotPosition(ss.str(),c,r,figure,RED,0.05);
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

            void run()
            {
                fun::PlanningResult latest_planning_result;
                if (planning_result_feed_->hasNext())
                {
                    
                }
            }
    };

  }
}