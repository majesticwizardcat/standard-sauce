#pragma once

#include <array>
#include <vector>
#include <string>

namespace consts {

static constexpr float ERROR = 0.0001f;
static constexpr float CLOSE_ERROR = 0.5f;
static constexpr float PI = 3.14159f;
static constexpr float TWO_PI = 2.0f * PI;
static constexpr float PI_OVER_TWO = PI * 0.5f;

}

typedef unsigned char uint8_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char byte_t;

namespace sauce {

//////////////////////////// COMMON DATA STRUCTS //////////////////////////// 

template <typename T, uint64_t Capacity> class StaticVector {
	static_assert(std::is_trivially_destructible_v<T>);

public:
	constexpr StaticVector()
			: m_size(0) { }

	constexpr StaticVector(std::initializer_list<T> initValues)
			: m_size(initValues.size()) {
		std::move(initValues.begin(), initValues.end(), m_array.begin());
	}

	constexpr StaticVector(const StaticVector& other)
			: m_size(other.m_size)
			, m_array(other.m_array) {
	}

	constexpr StaticVector(StaticVector&& other)
			: m_size(std::move(other.m_size))
			, m_array(std::move(other.m_array)) { }


	inline constexpr uint64_t size() const { return m_size; }
	inline constexpr const auto begin() const { return m_array.begin(); }
	inline constexpr const auto end() const { return m_array.begin() + m_size; }
	inline constexpr bool empty() const { return m_size == 0; }
	inline constexpr const T* data() const { return m_array.data(); }

	inline constexpr void push(const T& val) { m_array[m_size++] = val; }
	inline constexpr T pop() { return m_array[--m_size]; }
	inline constexpr void erase(const uint64_t index) { std::swap(m_array[index], m_array[--m_size]); }
	inline constexpr void clear() { m_size = 0; }
	inline constexpr T& addNew() { return m_array[m_size++]; }
	inline constexpr T& operator[](const uint64_t pos) { m_size = std::max(m_size, pos + 1); return m_array[pos]; }
	inline constexpr T* mutableData() { return m_array.data(); }

	inline constexpr StaticVector& operator=(StaticVector&& other) {
		m_array = std::move(other.m_array);
		m_size = std::move(other.m_size);
		return *this;
	}

private:
	uint64_t m_size;
	std::array<T, Capacity> m_array;
};

//////////////////////////// COMMON FUNCTIONS //////////////////////////// 

inline constexpr bool are_equal(float f0, float f1) {
	return std::abs(f0 - f1) < consts::ERROR;
}

inline constexpr bool are_not_equal(float f0, float f1) {
	return std::abs(f0 - f1) > consts::ERROR;
}

inline constexpr bool is_zero(float f) {
	return are_equal(f, 0.0f);
}

inline constexpr bool are_close(float f0, float f1) {
	return std::abs(f0 - f1) < consts::CLOSE_ERROR;
}

inline constexpr bool are_not_close(float f0, float f1) {
	return std::abs(f0 - f1) > consts::CLOSE_ERROR;
}

inline constexpr float fast_inv_sqrt(float f) {
	static constexpr float threehalves = 1.5f;
	const float x2 = f * 0.5f;

	long long i = *reinterpret_cast<long long*>(&f);
	i = 0x5f3759df - (i >> 1);
	f = *reinterpret_cast<float*>(&i);

	f = f * (threehalves - (x2 * f * f));
	f = f * (threehalves - (x2 * f * f));
	return f;
}

inline constexpr float fast_sqrt(float f) {
	return 1.0f / fast_inv_sqrt(f);
}


//////////////////////////// TOKENIZER //////////////////////////// 

template <uint32_t Size> class StaticString {
public:
	StaticString() = default;

	constexpr StaticString(const char* str) {
		strncpy(m_string, str, Size - 1);
		m_string[Size - 1] = '\0';
	}

	constexpr StaticString(const char* str, const uint32_t sizeOfStr) {
		const uint32_t finalSize = std::min(Size - 1, sizeOfStr);
		strncpy(m_string, str, finalSize);
		m_string[finalSize] = '\0';
	}

	inline constexpr void operator=(const char* str) {
		strncpy(m_string, str, Size);
		m_string[Size - 1] = '\0';
	}

	inline constexpr bool operator==(const char* str) const {
		return strncmp(str, m_string, Size) == 0;
	}

	inline constexpr bool operator==(StaticString str) const {
		return strncmp(str.m_string, m_string, Size) == 0;
	}

	inline constexpr const char* getString() const {
		return m_string;
	}

	inline constexpr uint32_t size() const {
		return strnlen(m_string, Size);
	}
	
private:
	char m_string[Size];
};

static constexpr uint32_t MAX_TOKEN_SIZE = 256;
typedef StaticString<MAX_TOKEN_SIZE> Token;

inline constexpr std::vector<Token> tokenize(const char* str, uint32_t size, const char delim = ' ') {
	std::vector<Token> tokens;

	uint32_t curWordStart = 0;
	for (uint32_t i = 0; i < size; ++i) {
		if (str[i] == delim) {
			if (i != curWordStart) {
				tokens.emplace_back(str + curWordStart, i - curWordStart);
			}

			curWordStart = i + 1;
		}
	}
	if (curWordStart != size) {
		tokens.emplace_back(str + curWordStart);
	}

	return tokens;
}

inline constexpr std::vector<Token> tokenize(const std::string& str, const char delim = ' ') {
	return sauce::tokenize(str.c_str(), str.size(), delim);
}

inline constexpr std::vector<Token> tokenize(const Token& str, const char delim = ' ') {
	return sauce::tokenize(str.getString(), str.size(), delim);
}

