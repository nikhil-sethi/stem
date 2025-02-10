#include <iostream>
#include <list>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <chrono>
#include <stem_msgs/target_viewpoint.h>

using namespace Eigen;
class Object{
    std::vector<TargetViewpoint> vpts;
};

void foo(std::vector<TargetViewpoint>& sample_vpts, int k){
    for (int i = 0; i < 10; ++i) {
        TargetViewpoint vpt(Vector3d(i * 0.1, i * 0.2, i * 0.3), 0.5 + k*i * 0.001);
        sample_vpts.push_back(vpt);
    }
}


void TEST_removeSimilarPosesFromList(){
    std::list<std::vector<TargetViewpoint>> myList;
    // // Example of adding elements
    // std::vector<PositionAndYaw> vec1;
    // vec1.push_back({Vector3d(1.0, 2.0, 3.0), 0.5});
    // vec1.push_back({Vector3d(1.1, 2.1, 3.1), 0.6}); // Similar to the first pose
    // vec1.push_back({Vector3d(4.0, 5.0, 6.0), 1.0});


    // std::vector<PositionAndYaw> vec2;
    // vec2.push_back({Vector3d(7.0, 8.0, 9.0), 1.5});
    // vec2.push_back({Vector3d(1.05, 2.05, 3.05), 0.55}); // Similar to the first pose in vec1
    // vec2.push_back({Vector3d(10.0, 11.0, 12.0), 2.0});

    for (int j =0;j<2;j++){
        std::vector<TargetViewpoint> sample_vpts;
        foo(sample_vpts, j*100);
        myList.push_back(sample_vpts);
    }
   
    // std::vector<*TargetViewpoint> vec3;
    // for (int i = 0; i < 30; ++i) {
    //     vec3.push_back(TargetViewpoint(Vector3d(i * 0.1, i * 0.2, i * 0.3), 0.5 + i * 0.001));
    // }
    //     std::vector<*TargetViewpoint> vec4;
    // for (int i = 0; i < 30; ++i) {
    //     vec4.push_back(TargetViewpoint(Vector3d(i * 0.1, i * 0.2, i * 0.3), 0.5 + i * 0.001));
    // }
    //     std::vector<*TargetViewpoint> vec5;
    // for (int i = 0; i < 30; ++i) {
    //     vec5.push_back(TargetViewpoint(Vector3d(i * 0.1, i * 0.2, i * 0.3), 0.5 + i * 0.001));
    // }
    //     std::vector<TargetViewpoint> vec6;
    // for (int i = 0; i < 30; ++i) {
    //     vec6.push_back(TargetViewpoint(Vector3d(i * 0.1, i * 0.2, i * 0.3), 0.5 + i * 0.001));
    // }

    // myList.push_back(vec1);
    // myList.push_back(vec2);
    // myList.push_back(vec3);
    // myList.push_back(vec4);
    // myList.push_back(vec5);
    // myList.push_back(vec6);


    // print list before
    
    std::cout << "Before: "<<std::endl; 
    for (const auto& vec : myList) {
        for (const auto& elem : vec) {
            std::cout << "Position: " << elem.pos_.transpose() << ", Yaw: " << elem.yaw_ << std::endl;
        }
    }

    std::cout<<std::endl;

    // Remove similar poses

    std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
    removeSimilarPosesFromList(myList);
    std::chrono::duration<double> dt_sam = std::chrono::high_resolution_clock::now() - start;
    printf("Time to remove: %f \n", dt_sam);
    
    printf("After: \n");
    // Print the updated list
    for (const auto& vec : myList) {
        for (const auto& elem : vec) {
            std::cout << "Position: " << elem.pos_.transpose() << ", Yaw: " << elem.yaw_ << std::endl;
        }
    }

}

void TEST_getColor(){
    Eigen::Matrix<double, 4, 4> colormap;
        colormap <<
        0 ,0,255,1, // blue
        0,255,0,1, // green
        255,255,0,1, //yellow
        255,0,0,1; // red
    colormap = colormap/255;
    Eigen::Vector3d pos;
    TargetViewpoint vpt(pos, 1.2, 2);
    TargetViewpoint vpt2(pos, 1.2, 5);

    Eigen::Vector4d color= vpt.getColor(0,10, colormap);
    Eigen::Vector4d color2= vpt2.getColor(0,10, colormap);

    std::cout<<color.transpose()<<std::endl;
    std::cout<<color2.transpose()<<std::endl;
}


// void TEST_removeSimilarPosesFromListWithObject(){
//     std::list<std::vector<TargetViewpoint>> myList;
 
//     std::vector<TargetViewpoint> vec1;
//     for (int i = 0; i < 10; ++i) {
//         vec1.push_back(TargetViewpoint(Vector3d(i * 0.1, i * 0.2, i * 0.3), 0.5 + i * 0.001));
//     }



//     std::vector<TargetViewpoint> vec2;
//     for (int i = 0; i < 10; ++i) {
//         vec2.push_back(TargetViewpoint(Vector3d(i * 0.1, i * 0.2, i * 0.3), 0.5 + i * 0.1));
//     }


//     myList.push_back(vec1);
//     myList.push_back(vec2);

//     // print list before
    
//     std::cout << "Before: "<<std::endl; 
//     for (const auto& vec : myList) {
//         for (const auto& elem : vec) {
//             std::cout << "Position: " << elem.pos_.transpose() << ", Yaw: " << elem.yaw_ << std::endl;
//         }
//     }

//     std::cout<<std::endl;

//     // Remove similar poses

//     std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
//     removeSimilarPosesFromList(myList);
//     std::chrono::duration<double> dt_sam = std::chrono::high_resolution_clock::now() - start;
//     printf("Time to remove: %f \n", dt_sam);
    
//     printf("After: \n");
//     // Print the updated list
//     for (const auto& vec : myList) {
//         for (const auto& elem : vec) {
//             std::cout << "Position: " << elem.pos_.transpose() << ", Yaw: " << elem.yaw_ << std::endl;
//         }
//     }

// }

int main() {

    TEST_removeSimilarPosesFromList();    

    TEST_getColor();

    return 0;
}
