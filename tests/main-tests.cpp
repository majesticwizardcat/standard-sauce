#include "sauce.hpp"

#include <iostream>

int main() {
	std::cout << "Starting main tests..." << '\n';

	assert(sauce::count_one_bits(1) == 1);
	assert(sauce::count_one_bits(3) == 2);
	assert(sauce::count_one_bits(0) == 0);
	assert(sauce::count_one_bits(69420) == 8);
	assert(sauce::count_one_bits(42069) == 7);

	assert(sauce::is_aligned_pow2(0, 2) == true);
	assert(sauce::is_aligned_pow2(1, 2) == false);
	assert(sauce::is_aligned_pow2(3, 4) == false);
	assert(sauce::is_aligned_pow2(66, 8) == false);
	assert(sauce::is_aligned_pow2(512, 512) == true);
	assert(sauce::is_aligned_pow2(2048, 512) == true);
	assert(sauce::is_aligned_pow2(512, 1) == true);
	assert(sauce::is_aligned_pow2(1, 1) == true);

	assert(sauce::get_aligned_pow2(0, 2) == 0);
	assert(sauce::get_aligned_pow2(1, 2) == 2);
	assert(sauce::get_aligned_pow2(1, 4) == 4);
	assert(sauce::get_aligned_pow2(10, 4) == 12);
	assert(sauce::get_aligned_pow2(10, 512) == 512);
	assert(sauce::get_aligned_pow2(512, 512) == 512);
	assert(sauce::get_aligned_pow2(127, 128) == 128);

	std::cout << "Done!" << '\n';
}

