

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

    class PlotGraphSearch
    {
        private:
            DLR_TS::PlotLab::FigureStubFactory fig_factory;

            /*void evaluate(int N, adore::env::OccupanyGrid* og, DLR_TS::PlotLab::AFigureStub* figure =nullptr)
            {
                int n = parallel_control.size();
                bool plot = false;
               for(int i=0 ; i<n ; ++i)        
               {
                   //plot = false;
                   //if(i==25) plot = true;
                integrate(N,og, &parallel_control[i],RED,figure, plot); 
                obj_F[i] = (states[L].back() );
               } 
               /*
               double s = states[S].back(); 
               int index = adore::mad::CubicPiecewiseFunction::findIndex(s ,pp_x);
               double r_psi = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_psi);
               double r_x = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_x);
               double r_y = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_y); 
               double l_dist = -sin(r_psi)* (r_x-states[X].back())+ cos(r_psi)* (r_y-states[Y].back());
               std::cout<<"\n"<<  r_x <<"\t"<<r_y <<"\t"<<r_psi;
               std::cout<<"\n"<<  states[X].back() <<"\t"<<states[Y].back() <<"\t"<<s<<"\t"<<l_dist;
               obj_F[n-1] +=   l_dist;  
               */           
                            
            /*}

            void initHorizon(int N, adore::env::OccupanyGrid* og,DLR_TS::PlotLab::AFigureStub* figure =nullptr)
            {
                obj_F.clear();
                obj_F.resize(N);
                d_obj_F.clear();
                d_obj_F.resize(N);
                parallel_control.clear();
                parallel_control.resize(N);
                control_back.clear();
                control_back.resize(N);
                control.clear();
                states[X].clear();
                states[Y].clear();
                states[PSI].clear();
                states[S].clear();
                states[L].clear();
                states[X].push_back(pre_trajectory[0].x);
                states[Y].push_back(pre_trajectory[0].y);
                states[PSI].push_back(pre_trajectory[0].psi);
                states[S].push_back(0.0);
                states[L].push_back(0.0); 
                double tmp_x[nX], x[nX];
                for(int i=0; i<nX; ++i) tmp_x[i] = states[i][0];
               // std::memcpy(&tmp_x[0],&x[0],nX*sizeof(double));                
                for(int i=0; i<N; ++i)        
                {
                    double s = states[S].back();
                    int index = adore::mad::CubicPiecewiseFunction::findIndex(s,pp_d);
                    double input = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_d);
                    control.push_back(input);
                    ralston(og, &tmp_x[0],input,&x[0],dt);
                    for(int j=0; j<nX; ++j) {states[j].push_back(x[j]);}
                    std::memcpy(&tmp_x[0],&x[0],nX*sizeof(double)); 
                }  
                
               // std::cout<<"\n"<<states[Y].size();                           

                

            }*/
        
        public:
            DLR_TS::PlotLab::AFigureStub* figure3;  
            DLR_TS::PlotLab::AFigureStub* figure4; 
            DLR_TS::PlotLab::AFigureStub* figure5;

            PlotGraphSearch()
            {

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

            /*static void optimize(int N, adore::env::OccupanyGrid* og,DLR_TS::PlotLab::AFigureStub* figure =nullptr)
            {
                   // createParallelSystems(N,&control);
                    evaluate(N,og,figure);
                    addEpsilon();
                    evaluate(N,og,figure);
                    gradient(N);
                    update_control(N);
                    integrate(N,og,&control,BLUE,figure,false); 
            }*/



            static void plotSoftRectangle(_Obstacle* obst,DLR_TS::PlotLab::AFigureStub* figure,std::string tag)
            {
                std::vector<double> x_v, y_v;
                double x,y,xt,yt ;
                for (double beta = 0.0; beta < 2*pi ; beta = beta + 0.1745)
                {
                    polar2Cartesian(x, y, get_softRectangle_r (beta, obst->length, obst->width), beta);
                    transformation(xt, yt,obst->alpha, x, y);
                    x_v.push_back(obst->x + xt);
                    y_v.push_back(obst->y + yt);
                    //std::cout<<"\n"<<x<<"\t"<<y;
                }
                figure->plot(tag,&x_v[0],&y_v[0],2.5,x_v.size(), GREEN);
            }

            static void plotEllipse(_Obstacle* obst,DLR_TS::PlotLab::AFigureStub* figure,std::string tag)
            {
                std::vector<double> x_v, y_v;
                double x,y,xt,yt ;
                for (double beta = 0.0; beta < 2*pi ; beta = beta + 0.1745)
                {
                    polar2Cartesian(x, y, get_ellipse_r (beta, obst->length, obst->width), beta);
                    transformation(xt, yt,obst->alpha, x, y);
                    x_v.push_back(obst->x + xt);
                    y_v.push_back(obst->y + yt);
                    //std::cout<<"\n"<<x<<"\t"<<y;
                }
                figure->plot(tag,&x_v[0],&y_v[0],2.5,x_v.size(), GREEN);
            }            
            void plotPolygon(_Obstacle* obst,DLR_TS::PlotLab::AFigureStub* figure,std::string tag)
            {
               //std::cout<<"\n****** "<<obst->vertices_x.size();
                figure->plot(tag,&obst->vertices_x[0],&obst->vertices_y[0],2.5,obst->vertices_x.size(), GREEN);

            }
            void PLOT(DLR_TS::PlotLab::AFigureStub* figure)
            {
                
                              
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
    };

  }
}