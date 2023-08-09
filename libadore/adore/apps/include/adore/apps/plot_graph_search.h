

namespace adore
{
  namespace apps
  {

    class PlotGraphSearch
    {
        private:
            DLR_TS::PlotLab::FigureStubFactory fig_factory;
            DLR_TS::PlotLab::AFigureStub* figure3;  
            DLR_TS::PlotLab::AFigureStub* figure4; 
            DLR_TS::PlotLab::AFigureStub* figure5;
        
        public:
            PlotGrapSearch ()
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
    }

  }
}