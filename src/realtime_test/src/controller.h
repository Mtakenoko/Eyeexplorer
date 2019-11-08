#ifndef __CONTROLLER
#define __CONTROLLER
#include <unistd.h>
#include <libts01.h>
#include <ktl.h>
// #include <QtCore/QVector>

class Controller
{

    //double dt;

public:
    double dt;
    TS01Config config;

    TS01InputData input;
    TS01OutputData output;

    int period;
    static bool flag_exit;
    double time;
    int status;

    Ktl::Wave uwave[TS01_AO_CH_NUM];
    double cntx[TS01_CNT_CH_NUM];
    double ssix[TS01_SSI_CH_NUM];

    Controller()
    {
        // printf("Controller cstr \n");
    }

    void *thread(void *);

    // QVector<double> g_time;
    // QVector<double> g_fx;
    // QVector<double> g_fy;
    // QVector<double> g_fz;
    // QVector<double> g_mx;
    // QVector<double> g_my;
    // QVector<double> g_mz;

    double current;
};

#endif
