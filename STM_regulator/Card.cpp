#include <fstream>
#include "Card.h"

using namespace std;




Card::Card(int PTR_, int ADC_CHANNEL_COUNT, string name, int PTW_) : name(name), PTR_(PTR_), PTW_(PTW_)
{
	cout << name << "  initialization..." << endl;
	tmr.set_to_zero();
	is_reading = 0;
	is_writing[0] = 0;
	is_writing[1] = 0;
}

void Card::BackstepZ() {
	double cur_h = cur_volt[0];
	for (double i = cur_h - 0.05; i > 0; i -= 0.00005) {
		SingleAnalogOut(i);
	}

	for (int k = 0; k < 6; k++) {
		SingleAnalogOut(5);
		for (double i = min((cur_h / 1.5 - 0.2 + k / 2), 5); i > 0; i -= 0.0003) {
			SingleAnalogOut(i);
		}
	}

}