#define SAUCE_DEBUG_OUTPUT

#include "sauce.hpp"

#include <iostream>

struct TestStruct {
	uint64_t a, b, c, d;
	float af, bf, cf, df;

	~TestStruct() {
		std::cout << "~TestStruct()" << '\n';
	}
};

sauce::RefCountedPtr<TestStruct> get() {
	sauce::RefCountedPtr<TestStruct> ptr (
		new TestStruct {
			.a = 1
			, .b = 2
			, .c = 3
			, .d = 4
			, .af = 1.0f
			, .bf = 2.0f
			, .cf = 3.0f
			, .df = 4.0f
		}
	);

	ptr->a = 99;

	return ptr;
}

int main() {
	{
		sauce::RefCountedPtr<TestStruct> ptr = get();
	}
}
