#include <ros/ros.h>
#include <bitset>

#include "owi_robot/generatecmd.h"

#define ELBOW_UP    "00010000"
#define ELBOW_DN    "00100000"
#define BYTE_SIZE   8
#define SPLITTER    "00001111"

bool cmdGenServer_callback(owi_robot::generatecmd::Request &req, owi_robot::generatecmd::Response &res){
    
    std::bitset<BYTE_SIZE> bset1(std::string(ELBOW_DN));
    // std::cout << bset1 << std::endl;
    
    std::bitset<BYTE_SIZE> splitter(std::string(SPLITTER));
    // std::cout << splitter << std::endl;

    auto right_half = std::bitset<BYTE_SIZE/2> ((bset1 & splitter).to_ulong());
    auto left_half = std::bitset<BYTE_SIZE/2> (((bset1 >> 4) & splitter).to_ulong());

    // std::cout << right_half << std::endl;
    // std::cout << left_half << std::endl;

    int b = right_half.to_ulong();
    int a = left_half.to_ulong();

    // std::cout << a << std::endl;
    // std::cout << b << std::endl;

    std::string byte1 = std::to_string(((10*a)+b));
    std::cout <<byte1 <<std::endl;

    res.str1 = byte1;
    res.str2 = "00";
    res.str3 = "00";








    return true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "cmdGenerate");
    ros::NodeHandle n;

    ros::ServiceServer cmdGen = n.advertiseService("cmdGenServer", cmdGenServer_callback);
    ros::spin();


    return 0;
}