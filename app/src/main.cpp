#include <cstdio>
#include <modm/platform.hpp>
#include <modm/processing.hpp>


#include "board.hpp"
#include "motor.hpp"
#include "serial.hpp"

using namespace modm::platform;
using namespace modm::literals;
using namespace std::chrono_literals;


using PwmTimer = modm::platform::Timer1;
using PwmU = GpioA8::Ch1;
using PwmV = GpioA9::Ch2;
using PwmW = GpioA10::Ch3;
using EnableU = GpioB3;
using EnableV = GpioB4;
using EnableW = GpioB5;

static const auto PWM_PERIOD = 42us;

static Adc adc;
static Usart1 serial;
static Motor<PwmU, PwmV, PwmW, PwmTimer> motor;
static SerialProcessing messageParser;

template<uint32_t MAX_SIZE>
class FloatFormatter {
private:
    char _fmt_buf[MAX_SIZE];
    int _size;
public: 
    void write(float x) {
        int len = snprintf(_fmt_buf, MAX_SIZE, "%.3f", (double)x);
        if(len > (int)MAX_SIZE - 1) {
            _size = MAX_SIZE - 1;
        } else {
            _size = len;
        }
    }

    uint8_t *data() { return (uint8_t*) _fmt_buf; }
    size_t size() { return _size; }
};

void send_state1(float encoder_angle, float commutation_angle, float mtr_current) {
    static const char *LABEL = "S1 ";
    // Size for three character label, three floats, newline + some 
    FloatFormatter<20> fmtbuf;

    serial.write((uint8_t *)LABEL, strlen(LABEL));
    fmtbuf.write(encoder_angle);
    serial.write(fmtbuf.data(), fmtbuf.size());
    serial.write(',');
    fmtbuf.write(commutation_angle);
    serial.write(fmtbuf.data(), fmtbuf.size());
    serial.write(',');
    fmtbuf.write(mtr_current);
    serial.write(fmtbuf.data(), fmtbuf.size());
    serial.write((uint8_t *)"\r\n", 2);
}

float read_angle() {
    uint16_t count = adc.readChannel(Adc::Channel::In2);
    float angle = 360.0f * (float)count / 4096.0f;
    return angle;
}

uint16_t setup_pwm_timer() {
    PwmTimer::connect<PwmU, PwmV, PwmW>();
    PwmTimer::enable();
    PwmTimer::setPeriod<Board::SystemClock>(PWM_PERIOD);
    PwmTimer::setMode(PwmTimer::Mode::UpCounter);
    PwmTimer::configureOutputChannel<PwmU>(
        PwmTimer::OutputCompareMode::Pwm,
        0,
        PwmTimer::PinState::Enable
    );
    PwmTimer::configureOutputChannel<PwmV>(
        PwmTimer::OutputCompareMode::Pwm,
        0,
        PwmTimer::PinState::Enable
    );
    PwmTimer::configureOutputChannel<PwmW>(
        PwmTimer::OutputCompareMode::Pwm,
        0,
        PwmTimer::PinState::Enable
    );
    PwmTimer::enableOutput();
    PwmTimer::start();

    EnableU::setOutput(true);
    EnableV::setOutput(true);
    EnableW::setOutput(true);
    return PwmTimer::getOverflow();
}


modm::PeriodicTimer timer{10ms};

int main() {

    // Setup PLL etc for 48MHz
    Board::SystemClock::enable();

    // Start systick
    SysTickTimer::initialize<Board::SystemClock>();

    adc.connect<GpioA2::In2>();
    adc.initialize<Board::SystemClock, Adc::ClockMode::Synchronous, 12_MHz>();

    serial.connect<GpioB7::Rx, GpioB6::Tx>();
    serial.initialize<Board::SystemClock, 115200>();

    uint16_t pwm_max = setup_pwm_timer();

    PwmTimer::setCompareValue<PwmU>(pwm_max / 2);

    motor.init(pwm_max);

    //motor.set_torque_target(-0.5);
    motor.set_target_pos(0.0);
    while(true) {
        if(timer.execute()) {
            float angle = read_angle();
            motor.run(10, angle * (float)std::numbers::pi / 180.f);
            send_state1(angle, motor.angle() * 180.f / (float)std::numbers::pi, motor.power());
        }

        uint8_t read_byte;
        if(serial.read(read_byte)) {
            Message rxMsg;
            if(messageParser.push_byte(read_byte, rxMsg)) {
                if(rxMsg.type() == CommandType::Calibrate) {
                    motor.start_calibration();
                } else if (rxMsg.type() == CommandType::Position) {
                    motor.set_target_pos((float)rxMsg.param_int(0) * 2.f * (float)std::numbers::pi / 360.f);
                }
            }
        }
    }
}
