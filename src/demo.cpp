#include "CoviewContext.h"

using namespace std;

int main()
{
    string refPath = "/home/dxz/Co-viewContext/data/zhuoer-global.pcd";
    string locPath = "/home/dxz/Co-viewContext/data/zhuoer1.pcd";
    string trajPath = "/home/dxz/Co-viewContext/data/zhuoer1.txt";
    string savePath = "/home/dxz/Co-viewContext/data/";

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

    cout << "Hello, World!" << endl;
    return 0;
}