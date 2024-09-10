#include "sauce.hpp"

#include <iostream>

int main() {
	std::cout << "Running static vector tests.. ";

	sauce::StaticVector<uint32_t, 128> v;

	std::cout << "Done!" << '\n';
}

