#include <iostream>
#include "NISwGSP_Stitching.h"
#include "TimeCalculator.h"

using namespace std;

int main(int argc, const char * argv[]) {

    Eigen::initParallel(); /* remember to turn off "Hardware Multi-Threading */
    cout << "nThreads = " << Eigen::nbThreads() << endl;
    cout << "[#Images : " << argc - 1 << "]" << endl;

    TimeCalculator timer;
    for(int i = 1; i < argc; ++i) {
        cout << "i = " << i << ", [Images : " << argv[i] << "]" << endl;
        MultiImages multi_images(argv[i], LINES_FILTER_WIDTH, LINES_FILTER_LENGTH);  //获取待拼接图像的全部信息
         
        timer.start();
        /* 2D */
        NISwGSP_Stitching niswgsp(multi_images);
        niswgsp.setWeightToAlignmentTerm(1);
        niswgsp.setWeightToLocalSimilarityTerm(0.75);
        niswgsp.setWeightToGlobalSimilarityTerm(6, 20, GLOBAL_ROTATION_2D_METHOD);

        niswgsp.writeImage(niswgsp.solve(BLEND_AVERAGE), BLENDING_METHODS_NAME[BLEND_AVERAGE]);		//平均值法融合
	  	
        niswgsp.writeImage(niswgsp.solve(BLEND_LINEAR),  BLENDING_METHODS_NAME[BLEND_LINEAR]);   //加权平均融合

      
        niswgsp.setWeightToAlignmentTerm(1);
        niswgsp.setWeightToLocalSimilarityTerm(0.75);
        niswgsp.setWeightToGlobalSimilarityTerm(6, 20, GLOBAL_ROTATION_3D_METHOD);
        niswgsp.writeImage(niswgsp.solve(BLEND_AVERAGE), BLENDING_METHODS_NAME[BLEND_AVERAGE]);
        niswgsp.writeImage(niswgsp.solve(BLEND_LINEAR),  BLENDING_METHODS_NAME[BLEND_LINEAR]);
		

        timer.end("[NISwGSP] " + multi_images.parameter.file_name);
    }
    return 0;
}
