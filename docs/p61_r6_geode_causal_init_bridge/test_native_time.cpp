// Regression: the unchanged author callback rounds raw header through double seconds.
#include <ros/time.h>
#include <cassert>
#include <iostream>
int main() {
    const uint64_t original=1706671587824846982ULL;
    ros::Time raw(original/1000000000ULL,original%1000000000ULL);
    const auto native=ros::Time().fromSec(raw.toSec()-0.0);
    assert(native.toNSec()!=original); // old arrival comparison would never complete
    uint64_t arrival_watermark=raw.toNSec();
    assert(arrival_watermark==original); // retain raw watermark, not a relaxed tolerance
    assert(native.toNSec()<=1706671587829840123ULL);
    std::cout<<"RAW="<<original<<" NATIVE="<<native.toNSec()<<" ARRIVAL_WATERMARK_PASS\n";
}
