#include "CoviewContext.h"

using namespace std;

int main(int argc,char*argv[])
{
    string refPath;
    string locPath;
    string trajPath;
    string savePath;

    if(argc == 5)
    {
        refPath = argv[1];
        locPath = argv[2];
        trajPath = argv[3];
        savePath = argv[4];
    }
    else
    {
        cout << "Usage: ./demo reference.pcd local.pcd trajectory.txt savePath" << endl;
        return -1;
    }

    CoviewContext coviewContext;
    coviewContext.referncePCDPath = refPath;
    coviewContext.localPCDPath = locPath;
    coviewContext.localTrajectoryPath = trajPath;
    coviewContext.saveBasePath = savePath;

    coviewContext.gridSize = 10.0;
    coviewContext.searchRadius = 25.0;
    coviewContext.downSampleSize = 0.75;
    coviewContext.icp_thr = 0.6;

    coviewContext.processReferenceCloud();
    coviewContext.processLocalCloud();

    return 0;
}