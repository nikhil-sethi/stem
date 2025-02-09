#include <stem_utils/utils.h>

void TEST_isPtInBox(){

    // CASE 1
    Eigen::Vector3d point(2,3,4);
    
    Eigen::Vector3d bbox_max(3,4,5);
    Eigen::Vector3d bbox_min(1,2,3);
    assert(isPtInBox(point, bbox_min, bbox_max) == true);

    // CASE 2: negative test
    Eigen::Vector3d point2(-1,3,-4);
    
    Eigen::Vector3d bbox_max2(3,4,5);
    Eigen::Vector3d bbox_min2(1,3,-3);
    assert(isPtInBox(point2, bbox_min2, bbox_max2) == false);

}

int main(int argc, char** argv) {

    TEST_isPtInBox();
    return 0;
}