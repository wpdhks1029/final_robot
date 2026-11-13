#include <bits/stdc++.h>
#define private public
#define protected public
#include "src/YDlidarDriver.h"
#undef protected
#undef private
using namespace ydlidar;
using namespace ydlidar::core::common;
int main() {
  const std::vector<uint8_t> packet = {170,85,64,40,25,176,187,9,138,98,102,28,42,68,0,0,67,0,0,56,0,0,46,212,63,54,48,67,40,52,71,40,76,75,69,96,70,93,0,0,104,0,0,116,128,39,116,176,39,25,0,0,25,0,0,131,0,0,54,0,0,108,0,0,132,4,38,138,0,38,171,248,37,174,240,37,192,248,37,148,28,38,105,68,38,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,38,0,0,5,0,0,4,0,0,9,192,4,6,0,0,68,0,0,53,204,155,42,0,0,31,0,0};
  YDlidarDriver d;
  d.setIntensityBit(8);
  d.setAutoIntensity(false);
  assert(d.calcCheckSum(packet));
  auto truncated=packet;truncated.pop_back();
  assert(!d.calcCheckSum(truncated));
  d.m_datas.push_back(YDlidarDriver::Pack(packet, 1, 8));
  d.setIntensityBit(16);
  assert(d.parsePoints());
  assert(d.scan_node_count==packet[3]);
  assert(d.scan_node_buf[34].dist==1216);
  d.scan_node_count=0;
  d.m_datas.push_back(YDlidarDriver::Pack(truncated, 1, 8));
  assert(d.parsePoints());assert(d.scan_node_count==0);
  d.setIntensityBit(8);d.setAutoIntensity(true);
  d.m_datas.push_back(YDlidarDriver::Pack(packet, 1, 8));
  auto corrupted=packet;corrupted[10]^=1;
  assert(!d.calcCheckSum(corrupted));assert(!d.calcCheckSum(corrupted));
  assert(d.m_intensityBit==0);
  assert(d.parsePoints());assert(d.scan_node_count==packet[3]);
  std::cout << "PASS: captured checksum, truncated checksum rejection, queued format switch, stored truncation, auto-detection switch with queued packet; no serial connection" << std::endl;
}
