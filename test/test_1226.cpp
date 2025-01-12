#include <iostream>
#include "referencepath/reference_path.h"
#include "tools/mathtools.h"



int main(){
    RefPath ref_path(2000, 5);
    SineInfo sine_info(3.5, 0.01);
    generateSinewavePath(100, ref_path, sine_info);
    // CalKappa(ref_path, 1);

    ref_path.showPath();

    return 0;
}

