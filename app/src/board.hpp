#include <modm/platform.hpp>



namespace Board
{
using namespace modm::literals;
using namespace modm::platform;

struct SystemClock
{
	static constexpr uint32_t Frequency = 48_MHz;
	static constexpr uint32_t Ahb = Frequency;
	static constexpr uint32_t Apb = Frequency;

	static constexpr uint32_t Adc1   = Apb;

	static constexpr uint32_t Spi1   = Apb;

	static constexpr uint32_t Usart1 = Apb;

	static constexpr uint32_t I2c1   = Apb;

	static constexpr uint32_t Timer1  = Apb;
	static constexpr uint32_t Timer2  = Apb;
	static constexpr uint32_t Timer3  = Apb;
	static constexpr uint32_t Timer14 = Apb;
	static constexpr uint32_t Timer16 = Apb;
	static constexpr uint32_t Timer17 = Apb;

	static bool inline
	enable()
	{
		Rcc::enableInternalClock();	// 8MHz
		// (internal clock / 2) * 12 = 48MHz
		const Rcc::PllFactors pllFactors{
			.pllMul = 12,
			.pllPrediv = 1  // only used with Hse
		};
		Rcc::enablePll(Rcc::PllSource::HsiDiv2, pllFactors);
		// set flash latency for 48MHz
		Rcc::setFlashLatency<Frequency>();
		// switch system clock to PLL output
		Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
		Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
		Rcc::setApbPrescaler(Rcc::ApbPrescaler::Div1);
		// update frequencies for busy-wait delay functions
		Rcc::updateCoreFrequency<Frequency>();

		return true;
	}
};
}
