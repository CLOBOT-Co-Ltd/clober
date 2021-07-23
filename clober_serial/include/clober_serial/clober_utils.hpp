#ifndef __CloberUtils_H__
#define __CloberUtils_H__

#define PI 3.141592

class CloberUtils{
    public:
        float toRPM(float w);
        float toVelocity(float rpm);
        float toRad(float enc, int ppr);
};


#endif //__CloberUtils_H__