#ifndef DEVICE_SIMSTAT_CU
#define DEVICE_SIMSTAT_CU

__device__ struct simStat
{
    double xsum;
    double ysum;
    double xxsum;
    double yysum;
    double xysum;
    double count;
    double sim;

    simStat()
    {
        xsum = 0.0;
        ysum = 0.0;
        xxsum = 0.0;
        yysum = 0.0;
        xysum = 0.0;
        count = 0.0;
        sim = 1.0;
    };

    __device__ simStat& operator=(const simStat& param)
    {
        xxsum = param.xxsum;
        yysum = param.yysum;
        xysum = param.xysum;
        xsum = param.xsum;
        ysum = param.ysum;
        count = param.count;
        return *this;
    };

    __device__ void computeSim()
    {
        // NCC
        double d = (xxsum - ((xsum * xsum) / count)) * (yysum - ((ysum * ysum) / count));
        if(d > 0.000000001)
        {
            sim = ((xysum - ((xsum * ysum) / count))) / sqrt(d);
            sim = 0.0 - sim;
        }
        else
        {
            sim = 1.0;
        };

        sim = max(sim, -1.0);
        sim = min(sim, 1.0);

        /*
        //MNCC
        float d = (xxsum - ( (xsum*xsum)/(float)count ) ) + (yysum - ( (ysum*ysum)/(float)count ));
        if (d > 0.0)
        {
                sim = ( 2.0 * ( xysum - ( (xsum*ysum)/(float)count ) ) ) /  d;
                sim = 0.0 - sim;
        }else
        {
                sim = 1.0;
        };
        */
    };

    __device__ int getVarianceX() { return (int)(xxsum / count - (xsum * xsum) / (count * count)); };

    __device__ int getVarianceY() { return (int)(yysum / count - (ysum * ysum) / (count * count)); };

    __device__ void update(float2& g)
    {
        count += 1;
        xsum += (double)g.x;
        ysum += (double)g.y;
        xxsum += (double)g.x * (double)g.x;
        yysum += (double)g.y * (double)g.y;
        xysum += (double)g.x * (double)g.y;
    };
};

#endif // DEVICE_SIMSTAT_CU
