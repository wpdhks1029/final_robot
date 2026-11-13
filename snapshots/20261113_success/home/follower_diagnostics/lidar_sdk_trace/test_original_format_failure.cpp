#include <bits/stdc++.h>
#define private public
#define protected public
#include "src/YDlidarDriver.h"
#undef protected
#undef private
using namespace ydlidar;
int main(){
const std::vector<uint8_t> packet = {170,85,64,40,25,176,187,9,138,98,102,28,42,68,0,0,67,0,0,56,0,0,46,212,63,54,48,67,40,52,71,40,76,75,69,96,70,93,0,0,104,0,0,116,128,39,116,176,39,25,0,0,25,0,0,131,0,0,54,0,0,108,0,0,132,4,38,138,0,38,171,248,37,174,240,37,192,248,37,148,28,38,105,68,38,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,38,0,0,5,0,0,4,0,0,9,192,4,6,0,0,68,0,0,53,204,155,42,0,0,31,0,0};
YDlidarDriver d;d.setIntensityBit(8);d.m_datas.push_back(YDlidarDriver::Pack(packet,1));d.setIntensityBit(16);
try { d.parsePoints(); }
catch(const std::out_of_range& e){std::cout<<"REPRODUCED original SDK exception: "<<e.what()<<std::endl;return 0;}
std::cerr<<"Expected exception not reproduced"<<std::endl;return 1;
}
