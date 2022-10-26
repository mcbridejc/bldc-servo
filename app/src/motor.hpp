#include <cstdint>
#include <cmath>
#include <numbers>
#include <modm/platform.hpp>
#include "fpm/fixed.hpp"
#include "fpm/math.hpp"

// Number of electrical revolutions per mechanical
// Note that number of motor poles is usually counted as twice this
// e.g. 14 poles -> 7:1 electrical/mechanical ratio
static constexpr int NPOLES = 7;
static constexpr std::size_t COMM_TABLE_SIZE = 120;

static constexpr fpm::fixed_16_16 DEFAULT_CALIBRATION_SPEED {4.0};
static constexpr uint32_t DEFAULT_CALIBRATION_WAIT_US = 2 * 1000000;
static constexpr fpm::fixed_8_24 DEFAULT_CALIBRATION_POWER {0.8};
// Calibration which aligns motor commutation angle with encoder angle
static constexpr fpm::fixed_16_16 MOTOR_ZERO_CAL {246.388f * (float)std::numbers::pi / 180.0f};

static constexpr fpm::fixed_16_16 VEL_GAIN_P { 1.0 };
static constexpr fpm::fixed_16_16 VEL_GAIN_I { 0.3 };
static constexpr fpm::fixed_16_16 VEL_GAIN_D { 1.0 };
static constexpr fpm::fixed_16_16 IMAX { 0.2 };
static constexpr fpm::fixed_16_16 POS_GAIN_P { 0.1 };
static constexpr fpm::fixed_16_16 POS_GAIN_D { 0.0 };
static constexpr fpm::fixed_16_16 ACCEL { 10.0 };

template<typename T>
T wrap_radians(T x) {
    while(true) {
        if(x > T::pi()) {
            x -= 2 * T::pi();
        } else if (x < -T::pi()) {
            x += 2 * T::pi();
        } else {
            return x;
        }
    }
}

template<typename T>
T wrap_radians_360(T x) {
    while(true) {
        if(x > 2 * T::pi()) {
            x -= 2 * T::pi();
        } else if (x < T(0)) {
            x += 2 * T::pi();
        } else {
            return x;
        }
    }
}

template<typename U, typename V, typename W, typename PwmTimer>
class Motor {
    enum MotorState {
        Idle,
        Control,
        Torque,
        AngleCalibration,
    };

    int16_t commTable[COMM_TABLE_SIZE];
    MotorState motorState;
    uint32_t calibrationWaitTime;
    fpm::fixed_16_16 calibrationAngle;
    bool calibrationReturning;
    uint16_t pwmMax;
    fpm::fixed_16_16 commutationAngle;
    fpm::fixed_8_24 motorPower;

    fpm::fixed_16_16 targetAngle;
    fpm::fixed_8_24 targetTorque;
    fpm::fixed_16_16 lastPosErr;
    fpm::fixed_16_16 lastVelErr;
    fpm::fixed_16_16 lastCmdVel;
    fpm::fixed_16_16 velIntegral;
    fpm::fixed_16_16 lastEncoderAngle;

public:
    /** Initialize the motor controller
     * 
     * pwm_max is the timer output value corresponding to 100% duty cycle
     */
    void init(uint16_t pwm_max) {
        for(uint32_t i=0; i<COMM_TABLE_SIZE; i++) {
            float theta = 2 * (float)std::numbers::pi * (float)i / (float)COMM_TABLE_SIZE;
            commTable[i] = (int16_t) (std::sin(theta) / 2.0  * pwm_max); 
        }

        pwmMax = pwm_max;
    }

    void start_calibration() {
        motorState = MotorState::AngleCalibration;
        calibrationWaitTime = 0;
        calibrationReturning = false;
        calibrationAngle = fpm::fixed_16_16(0);
    }
    
    void run(uint32_t elapsed_us, float encoder_angle) {
        if(motorState == MotorState::AngleCalibration) {
            runAngleCalibration(elapsed_us);
        } else if(motorState == MotorState::Control) {
            runControl(elapsed_us, encoder_angle);
        } else if(motorState == MotorState::Torque) {
            runTorque(encoder_angle);
        } else {
            runIdle();
        }
    }

    void set_target_pos(float angle) {
        targetAngle = fpm::fixed_16_16(angle);
        motorState = MotorState::Control;
    }

    void set_torque_target(float t) {
        targetTorque = fpm::fixed_8_24(t);
        motorState = MotorState::Torque;
    }

    float angle() {
        return static_cast<float>(commutationAngle);
    }

    float power() {
        return static_cast<float>(motorPower);
    }
    
private:

    void setCommutationAngle(fpm::fixed_16_16 angle, fpm::fixed_8_24 mag) {
        uint32_t iu = static_cast<uint32_t>(COMM_TABLE_SIZE * angle / fpm::fixed_16_16::two_pi()) % COMM_TABLE_SIZE;
        uint32_t iv = (iu + COMM_TABLE_SIZE / 3) % COMM_TABLE_SIZE;
        uint32_t iw = (iv + COMM_TABLE_SIZE / 3) % COMM_TABLE_SIZE;

        if(mag < fpm::fixed_8_24{0.0}) { 
            mag = fpm::fixed_8_24{0.0};
        } 
        if(mag > fpm::fixed_8_24{1.0}) {
            mag = fpm::fixed_8_24{1.0};
        }
        fpm::fixed_16_16 mag16 = static_cast<fpm::fixed_16_16>(mag);
        PwmTimer::template setCompareValue<U>((uint16_t)(pwmMax/2 + static_cast<int16_t>(commTable[iu] * mag16)));
        PwmTimer::template setCompareValue<V>((uint16_t)(pwmMax/2 + static_cast<int16_t>(commTable[iv] * mag16)));
        PwmTimer::template setCompareValue<W>((uint16_t)(pwmMax/2 + static_cast<int16_t>(commTable[iw] * mag16)));

        commutationAngle = angle;
        motorPower = mag;
    }

    void setTorque(fpm::fixed_16_16 encoder_angle, fpm::fixed_8_24 torque) {
        auto adjusted_angle = NPOLES * (encoder_angle - MOTOR_ZERO_CAL);

        if(torque > fpm::fixed_8_24{0.0}) {
            adjusted_angle += fpm::fixed_16_16::half_pi();
        } else {
            adjusted_angle -= fpm::fixed_16_16::half_pi();
        }
        setCommutationAngle(wrap_radians_360(adjusted_angle), fpm::abs(torque));
    }

    void runAngleCalibration(uint32_t elapsed_ms) {
        auto step = DEFAULT_CALIBRATION_SPEED * fpm::fixed_16_16(elapsed_ms) / fpm::fixed_16_16(1000);
            setCommutationAngle(fpm::fixed_16_16(0), DEFAULT_CALIBRATION_POWER);
        if(calibrationWaitTime < DEFAULT_CALIBRATION_WAIT_US) {
            calibrationWaitTime += elapsed_ms * 1000;
        } else if(!calibrationReturning) {
            if(calibrationAngle < 7 * fpm::fixed_16_16::two_pi()) {
                calibrationAngle += step;
                setCommutationAngle(wrap_radians_360(calibrationAngle), DEFAULT_CALIBRATION_POWER);
            } else {
                calibrationReturning = true;
            }
        } else {
            if(calibrationAngle > fpm::fixed_16_16{0.0}) {
                calibrationAngle -= step;
                setCommutationAngle(wrap_radians_360(calibrationAngle), DEFAULT_CALIBRATION_POWER);
            } else {
                motorState = MotorState::Idle;
            }
        }
    }

    void runControl(uint32_t elapsed_ms, float encoder_angle) { 
        // Run position controller
        fpm::fixed_16_16 angle = fpm::fixed_16_16(encoder_angle);
        fpm::fixed_16_16 err = wrap_radians(targetAngle - angle);
        fpm::fixed_16_16 cmd_vel = POS_GAIN_P * err + POS_GAIN_D * (err - lastPosErr);
   

        fpm::fixed_16_16 max_slew = ACCEL * elapsed_ms / 1000;
        if(cmd_vel > lastCmdVel + max_slew) {
            cmd_vel = lastCmdVel + max_slew;
        } else if(cmd_vel < lastCmdVel - max_slew) {
            cmd_vel = lastCmdVel - max_slew;
        }
        lastCmdVel = cmd_vel;

        fpm::fixed_16_16 cur_vel = (angle - lastEncoderAngle) * 1000 / elapsed_ms;
        fpm::fixed_16_16 vel_err = cmd_vel - cur_vel;
        velIntegral += err * VEL_GAIN_I;
        if(velIntegral > IMAX) {
            velIntegral = IMAX;
        } else if(velIntegral < -IMAX) {
            velIntegral = -IMAX;
        }
        fpm::fixed_16_16 mtr_cmd;// = VEL_GAIN_P * vel_err + velIntegral + VEL_GAIN_D * ( - lastVelErr);

        mtr_cmd = VEL_GAIN_P * err + velIntegral + VEL_GAIN_D * (err - lastPosErr);
        lastPosErr = err;
        if(mtr_cmd > fpm::fixed_16_16{1.0}) {
            mtr_cmd = fpm::fixed_16_16{1.0};
        } else if(mtr_cmd < fpm::fixed_16_16(-1.0)) {
            mtr_cmd = fpm::fixed_16_16{-1.0};
        }
        setTorque(static_cast<fpm::fixed_16_16>(angle), fpm::fixed_8_24(mtr_cmd));
        // auto adjusted_angle = 7 * (static_cast<fpm::fixed_16_16>(angle) - MOTOR_ZERO_CAL);
        // if(mtr_cmd > fpm::fixed_8_24 {0}) {
        //     setCommutationAngle(wrap_radians_360(adjusted_angle + fpm::fixed_16_16::half_pi()), mtr_cmd);
        // } else {
        //     setCommutationAngle(wrap_radians_360(adjusted_angle - fpm::fixed_16_16::half_pi()), -1 * mtr_cmd);
        // }
    }

    void runTorque(float encoder_angle) {
        setTorque(fpm::fixed_16_16(encoder_angle), targetTorque);
    }

    void runIdle() {
        PwmTimer::template setCompareValue<U>(0);
        PwmTimer::template setCompareValue<V>(0);
        PwmTimer::template setCompareValue<W>(0);
        commutationAngle = fpm::fixed_16_16{0.0};
        motorPower = fpm::fixed_8_24{0.0};
    }
};
