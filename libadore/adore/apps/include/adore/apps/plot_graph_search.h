

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



            /*void initHorizon(int N, adore::env::OccupanyGrid* og,DLR_TS::PlotLab::AFigureStub* figure =nullptr)
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

                        void optimize(int N, adore::env::OccupanyGrid* og,DLR_TS::PlotLab::AFigureStub* figure =nullptr)
            {
                   // createParallelSystems(N,&control);
                    evaluate(N,og,figure);
                    addEpsilon();
                    evaluate(N,og,figure);
                    gradient(N);
                    update_control(N);
                    integrate(N,og,&control,BLUE,figure,false); 
            }
    };

  }
}