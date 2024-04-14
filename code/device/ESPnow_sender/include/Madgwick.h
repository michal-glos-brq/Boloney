//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static float invSqrt(float x);
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    char anglesComputed;
    void computeAngles();
    float roll;
    float yaw;
    float pitch;
    float beta;	
//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    //float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    //float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    //float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }

    void setOrientation(float roll, float pitch, float yaw) {
        // Convert Euler angles (roll, pitch, yaw) into quaternion
        float cy = cos(yaw * 0.5);
        float sy = sin(yaw * 0.5);
        float cr = cos(roll * 0.5);
        float sr = sin(roll * 0.5);
        float cp = cos(pitch * 0.5);
        float sp = sin(pitch * 0.5);

        q0 = cy * cr * cp + sy * sr * sp;
        q1 = cy * sr * cp - sy * cr * sp;
        q2 = cy * cr * sp + sy * sr * cp;
        q3 = sy * cr * cp - cy * sr * sp;
        anglesComputed = 0; // Force recomputation of Euler angles when needed
    }
};
#endif