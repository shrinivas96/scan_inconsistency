#include <g2o/types/data/raw_laser.h>
#include <g2o/types/slam2d/se2.h>
#include <consist/visibility.h>

int main(int argc, char const *argv[])
{
    g2o::RawLaser myscan;
    consist::Visibility myvisible;
    std::cout << "The includes worked!!!" << std::endl;
    return 0;
}
