#pragma once

#include <array>
#include <algorithm>
#include <bit>
#include <cassert>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace consts {

static constexpr float ERROR = 0.0001f;
static constexpr float CLOSE_ERROR = 0.5f;
static constexpr float PI = 3.14159f;
static constexpr float TWO_PI = 2.0f * PI;
static constexpr float PI_OVER_TWO = PI * 0.5f;

static constexpr uint64_t KILO_BYTE = 1024;

}

#ifdef SAUCE_DEBUG_OUTPUT
#define SAUCE_DEBUG_PRINT(X) std::cout << X << '\n'
#else
#define SAUCE_DEBUG_PRINT(X)
#endif

typedef char byte_t;

namespace sauce {

//////////////////////////// MEMORY EXTENSIONS //////////////////////////// 

// Non-atomic ref counted ptr
template <typename T> class RefCountedPtr {
public:
	inline RefCountedPtr()
			: m_ptr(nullptr)
			, m_refCounter(nullptr) { SAUCE_DEBUG_PRINT("RefCountedPtr()"); }

	inline RefCountedPtr(T* ptr)
			: m_ptr(ptr)
			, m_refCounter(new uint64_t(1)) { SAUCE_DEBUG_PRINT("RefCountedPtr(T* ptr)"); }
	
	inline RefCountedPtr(const RefCountedPtr<T>& other)
			: m_ptr(other.m_ptr)
			, m_refCounter(other.m_refCounter) {
		copyFromPtr(other);

		if (!m_refCounter) {
			SAUCE_DEBUG_PRINT("RefCountedPtr(const RefCountedPtr<T>&) -> Counter: nullptr");
		}
		else {
			SAUCE_DEBUG_PRINT("RefCountedPtr(const RefCountedPtr<T>&) -> Counter: " << *m_refCounter);
		}
	}

	inline RefCountedPtr(RefCountedPtr<T>&& other)
			: m_ptr(other.m_ptr)
			, m_refCounter(other.m_refCounter) {
		SAUCE_DEBUG_PRINT("RefCountedPtr(RefCountedPtr<T>&&)");
		other.m_ptr = nullptr;
		other.m_refCounter = nullptr;
	}
	
	inline ~RefCountedPtr() {
		if (!m_refCounter) {
			assert(!m_ptr);
			SAUCE_DEBUG_PRINT("~RefCountedPtr() -> Counter: nullptr");
			return;
		}

		SAUCE_DEBUG_PRINT("~RefCountedPtr() -> Counter: " << *m_refCounter);

		assert(m_refCounter);
		assert(*m_refCounter > 0);

		{
			uint64_t& counter = *m_refCounter;
			counter--;
		}

		if (m_ptr) {
			if (*m_refCounter == 0) {
				SAUCE_DEBUG_PRINT("Deleting pointer");
				delete m_ptr;
			}

			m_ptr = nullptr;
		}

		if (*m_refCounter == 0) {
			SAUCE_DEBUG_PRINT("Deleting ref counter");
			delete m_refCounter;
			m_refCounter = nullptr;
		}
	}

	inline RefCountedPtr<T> operator=(const RefCountedPtr<T>& other) {
		copyFromPtr(other);
		if (!m_refCounter) {
			SAUCE_DEBUG_PRINT("RefCountedPtr(const RefCountedPtr<T>&) -> Counter: nullptr");
		}
		else {
			SAUCE_DEBUG_PRINT("RefCountedPtr(const RefCountedPtr<T>&) -> Counter: " << *m_refCounter);
		}

		return *this;
	}

	inline RefCountedPtr<T> operator=(RefCountedPtr<T>&& other) {
		SAUCE_DEBUG_PRINT("operator=(RefCounterPtr<T>&&)");
		m_ptr = other.m_ptr;
		m_refCounter = other.m_refCounter;

		other.m_ptr = nullptr;
		other.m_refCounter = nullptr;
		return *this;
	}

	inline T* get() const { return m_ptr; }
	inline T& operator*() const { return *m_ptr; }
	inline T* operator->() const { return m_ptr; }

	inline operator bool() const { return m_ptr != nullptr; }

private:
	T* m_ptr;
	uint64_t* m_refCounter;

	void copyFromPtr(const RefCountedPtr<T>& other) {
		m_ptr = other.m_ptr;
		m_refCounter = other.m_refCounter;

		if (m_refCounter) {
			uint64_t& counter = *m_refCounter;
			counter++;
		}
		assert((m_ptr && m_refCounter) || (!m_ptr && !m_refCounter));
	}
};

template <typename T, typename ...Args> inline static RefCountedPtr<T> make_ref_counted(Args... args) { return RefCountedPtr<T>(new T(args...)); }

class RefCountedObject {
public:
	RefCountedObject(RefCountedObject&& other) = delete; // only allow copy

	inline RefCountedObject()
			: m_refCounter(new uint64_t(1)) { }
	
	inline RefCountedObject(const RefCountedObject& other)
			: m_refCounter(other.m_refCounter) {
		increaseRef();
	}

	virtual inline ~RefCountedObject() {
		assert(m_refCounter);
		assert(*m_refCounter > 0);
		uint64_t& counter = *m_refCounter;
		counter--;

		if (counter == 0) {
			delete m_refCounter;
		}

		m_refCounter = nullptr;
	}

	RefCountedObject operator=(const RefCountedObject& other) {
		assert(!m_refCounter);
		m_refCounter = other.m_refCounter;
		increaseRef();
		return *this;
	}

protected:
	inline bool isLastRef() const { assert(m_refCounter); return *m_refCounter <= 1; }

private:
	uint64_t* m_refCounter;

	inline void increaseRef() {
		assert(m_refCounter);
		uint64_t& counter = *m_refCounter;
		counter++;
	}
};

//////////////////////////// DATA STRUCTS //////////////////////////// 

template <typename T, uint64_t Capacity> class StaticVector {
	static_assert(std::is_trivially_destructible_v<T>);

public:
	constexpr StaticVector()
			: m_size(0)
			, m_array{} { }

	constexpr StaticVector(const std::initializer_list<T> initValues)
			: m_size(initValues.size())
			, m_array{} {
		for (uint64_t i = 0; i < m_size; ++i) {
			m_array[i] = *(initValues.begin() + i);
		}
	}

	constexpr StaticVector(const StaticVector& other)
			: m_size(other.m_size)
			, m_array(other.m_array) {
	}

	constexpr StaticVector(StaticVector&& other)
			: m_size(std::move(other.m_size))
			, m_array(std::move(other.m_array)) { }


	constexpr uint64_t size() const { return m_size; }
	constexpr auto begin() const { return m_array.begin(); }
	constexpr auto end() const { return m_array.begin() + m_size; }
	constexpr bool empty() const { return m_size == 0; }
	constexpr const T* data() const { return m_array.data(); }
	constexpr const T& operator[](const uint64_t pos) const { assert(pos < m_size); return m_array[pos]; }

	constexpr auto begin() { return m_array.begin(); }
	constexpr auto end() { return m_array.begin() + m_size; }
	constexpr void push_back(const T& val) { m_array[m_size++] = val; }
	template <typename ...Args> constexpr void emplace_back(Args... args) { m_array[m_size++] = T(args...); }
	constexpr T pop_back() { return m_array[--m_size]; }
	constexpr void erase(const uint64_t index) { std::swap(m_array[index], m_array[--m_size]); }
	constexpr void clear() { m_size = 0; }
	constexpr T& addNew() { return m_array[m_size++]; }
	constexpr T& operator[](const uint64_t pos) { m_size = std::max(m_size, pos + 1); return m_array[pos]; }

	constexpr StaticVector& operator=(StaticVector&& other) {
		m_array = std::move(other.m_array);
		m_size = std::move(other.m_size);
		return *this;
	}

private:
	uint64_t m_size;
	std::array<T, Capacity> m_array;
};

template <uint64_t Size> class RingBufferIndexGenerator {
public:
	constexpr RingBufferIndexGenerator()
			: m_nextIndex(0)
			, m_releasedIndex(0) { }
	
	constexpr uint64_t getNext() {
		assert(hasAvailable());
		return m_nextIndex++ % Size;
	}

	constexpr void release() {
		assert((m_releasedIndex + 1) % Size <= m_nextIndex);
		m_releasedIndex = (m_releasedIndex + 1) % Size;
	}

	constexpr bool hasAvailable() const { 
		return (m_nextIndex + 1) % Size != m_releasedIndex;
	}

private:
	uint64_t m_nextIndex;
	uint64_t m_releasedIndex;
};

template <typename T, uint64_t Capacity, typename HashFunction, typename Comparer> class StaticIndexedHashMap {
public:
	constexpr StaticIndexedHashMap()
	  		: m_size(0)
			, m_array{} { }
	
	constexpr uint64_t add(const T& value) {
		assert(m_size < Capacity);

		const uint64_t hash = std::max(1ull, HashFunction::hash(value));
		uint64_t index = hash % Capacity;
		for (; ; index = (index + 1) % Capacity) {
			HashValuePair& pair = m_array[index];
			if (pair.hash == hash) {
				if (Comparer::areEqual(pair.value, value)) {
					break;
				}
			}
			else if (pair.hash == 0) {
				pair.hash = hash;
				pair.value = value;
				m_size++;
				break;
			}
		}

		return index;
	}

	constexpr const T& getFromIndex(const uint64_t index) const {
		assert(index < Capacity);
		return m_array[index].value;
	}

private:
	struct HashValuePair {
		uint64_t hash;
		T value;
	};

	uint64_t m_size;
	std::array<HashValuePair, Capacity> m_array;
};

//////////////////////////// FUNCTIONS //////////////////////////// 

static constexpr bool are_equal(const float f0, const float f1) {
	return std::abs(f0 - f1) < consts::ERROR;
}

static constexpr bool are_not_equal(const float f0, const float f1) {
	return std::abs(f0 - f1) > consts::ERROR;
}

static constexpr bool is_zero(const float f) {
	return are_equal(f, 0.0f);
}

static constexpr bool are_close(const float f0, const float f1) {
	return std::abs(f0 - f1) < consts::CLOSE_ERROR;
}

static constexpr bool are_not_close(const float f0, const float f1) {
	return std::abs(f0 - f1) > consts::CLOSE_ERROR;
}

static constexpr float fast_inv_sqrt(const float r) {
	static_assert(std::numeric_limits<float>::is_iec559); // (enable only on IEEE 754)

	const float halfR = 0.5f * r;
	const float y = std::bit_cast<float>(0x5f3759df - (std::bit_cast<std::uint32_t>(r) >> 1));
	const float f = y * (1.5f - (halfR * y * y));
	return f * (1.5f - (halfR * f * f));
}

static constexpr float fast_sqrt(const float f) {
	return 1.0f / fast_inv_sqrt(f);
}

static constexpr uint8_t count_one_bits(const uint64_t value) {
	uint8_t bits = 0;
	for (uint64_t i = 0; i < 64; ++i) {
		bits += ((value >> i) & 0x1);
	}

	return bits;
}

static constexpr bool is_aligned_pow2(const uint64_t value, const uint64_t alignment) {
	assert(alignment > 0);
	assert(count_one_bits(alignment) == 1);

	const uint64_t alignmentMask = alignment - 1;
	return alignmentMask == 0 || (value & alignmentMask) == 0;
}

static constexpr uint64_t get_aligned_pow2(const uint64_t value, const uint64_t alignment) {
	assert(alignment > 1);
	assert(count_one_bits(alignment) == 1);

	const uint64_t alignmentMask = alignment - 1;
	const uint64_t pad = (value & alignmentMask) > 0 ? alignment : 0; 
	return (value & ~alignmentMask) + pad;
}

inline static bool parse_whole_file(const std::string_view fileLocation, std::string& output) {
	std::ifstream file(fileLocation);

	if (!file) {
		std::cout << "Failed to open file: " << fileLocation << '\n';
		return false;
	}

	std::stringstream buffer;
	buffer << file.rdbuf();
	output = buffer.str();

	return true;
}

inline static bool save_to_file(const std::string& str, const std::string_view fileLocation) {
	std::ofstream file(fileLocation);

	if (!file) {
		std::cout << "Failed to open file for write: " << fileLocation << '\n';
		return false;
	}

	file.write(str.data(), str.size());
	return true;
}

static constexpr std::string_view trimSpacesLeft(std::string_view str) {
	str.remove_prefix(std::min(str.find_first_not_of(" "), str.size()));
	return str;
}

//////////////////////////// TOKENIZER //////////////////////////// 

template <typename OutputType> static constexpr void tokenize(OutputType& tokensOut, const char* str, const uint64_t size, const char delim = ' ', const uint64_t maxTokens = std::numeric_limits<uint64_t>::max()) {
	tokensOut.clear();

	uint64_t curWordStart = 0;
	for (uint64_t i = 0; i < size; ++i) {
		if (str[i] == delim) {
			if (i != curWordStart) {
				tokensOut.emplace_back(str + curWordStart, i - curWordStart);

				if (tokensOut.size() >= maxTokens) {
					return;
				}
			}

			curWordStart = i + 1;
		}
	}
	if (curWordStart != size) {
		tokensOut.emplace_back(str + curWordStart, size - curWordStart);
	}
}

template <uint64_t NumOfTokens> static constexpr StaticVector<std::string_view, NumOfTokens> tokenize(const std::string_view str, const char delim = ' ') {
	StaticVector<std::string_view, NumOfTokens> tokens;
	tokenize(tokens, str.data(), str.size(), delim, NumOfTokens);
	return tokens;
}

static constexpr std::vector<std::string_view> tokenize(const std::string_view str, const char delim = ' ') {
	static constexpr uint64_t minTokens = 32;

	std::vector<std::string_view> tokens;
	tokens.reserve(minTokens);
	tokenize(tokens,  str.data(), str.size(), delim);
	return tokens;
}

//////////////////////////// BYTE IO //////////////////////////// 

class Buffer {
public:
	Buffer(void* buffer, const uint64_t capacity)
			: m_buffer(reinterpret_cast<byte_t*>(buffer))
			, m_bufferCapacity(capacity)
			, m_currentBufferSize(0) { }

	virtual constexpr void reset() {
		m_currentBufferSize = 0;
	}

protected:
	byte_t* m_buffer;
	uint64_t m_bufferCapacity;
	uint64_t m_currentBufferSize;
};

class BufferReader : public Buffer {
public:
	BufferReader(byte_t* buffer, const uint64_t capacity, const uint64_t bytesAvailable = 0)
			: Buffer(buffer, capacity)
			, m_bufferReadEnd(bytesAvailable) { }

	inline void fill(std::istream& stream) {
		const uint64_t bytesToRead = m_bufferCapacity - m_bufferReadEnd;
		byte_t* readPtr = m_buffer + m_bufferReadEnd;
		stream.read(readPtr, bytesToRead);
		m_bufferReadEnd += stream.gcount();
		assert(m_bufferReadEnd <= m_bufferCapacity);
	}

	inline void resetAndFill(std::istream& stream) {
		reset();
		fill(stream);
	}

	template <typename T> inline T read() {
		T result;
		readBytesToBuffer(reinterpret_cast<byte_t*>(result), sizeof(result));
		return result;
	}

	inline void readBytesToBuffer(byte_t* bufferOut, const uint64_t bytesToRead) {
		assert(m_currentBufferSize + bytesToRead <= m_bufferReadEnd);
		memcpy(bufferOut, m_buffer + m_currentBufferSize, bytesToRead);
		m_currentBufferSize += bytesToRead;
	}

	inline void readAll(byte_t* bufferOut) {
		readBytesToBuffer(bufferOut, getBytesAvailable());
	}

	constexpr uint64_t getBytesAvailable() const {
		return m_bufferReadEnd - m_currentBufferSize;
	}

	constexpr void reset() override {
		Buffer::reset();
		m_bufferReadEnd = 0;
	}

private:
	uint64_t m_bufferReadEnd;
};

template <uint64_t Size> class StaticBufferReader : public BufferReader {
public:
	StaticBufferReader()
			: BufferReader(nullptr, Size, 0) {
		m_buffer = m_staticBuffer.data();
	}

private:
	std::array<byte_t, Size> m_staticBuffer;
};

class DynamicBufferReader : public BufferReader {
public:
	DynamicBufferReader(const uint64_t bufferSize)
			: BufferReader(nullptr, bufferSize) {
		m_dynamicBuffer = std::make_unique<byte_t[]>(bufferSize);
		m_buffer = m_dynamicBuffer.get();
	}

private:
	std::unique_ptr<byte_t[]> m_dynamicBuffer;
};

class BufferWriter : public Buffer {
public:
	BufferWriter(void* buffer, const uint64_t capacity)
			: Buffer(buffer, capacity) { }

	template <typename T> inline void write(const T& obj) {
		const byte_t* objAsBytePtr = reinterpret_cast<const byte_t*>(obj);
		writeBytes(objAsBytePtr, sizeof(T));
	}

	inline virtual void writeBytes(const byte_t* bytes, const uint64_t numOfBytes) {
		assert(m_currentBufferSize + numOfBytes <= m_bufferCapacity);
		memcpy(m_buffer + m_currentBufferSize, bytes, numOfBytes);
		m_currentBufferSize += numOfBytes;
	}

	template <typename T> constexpr bool canWrite(const T& obj) const {
		return canWrite(sizeof(obj));
	}

	inline virtual constexpr bool canWrite(const uint64_t numOfBytes) const {
		return m_currentBufferSize + numOfBytes <= m_bufferCapacity;
	}

	constexpr uint64_t bytesWritten() const {
		return m_currentBufferSize;
	}

	constexpr uint64_t availableSpace() const {
		return m_bufferCapacity - m_currentBufferSize;
	}

	inline void flushToStream(std::ostream& stream) const {
		stream.write(m_buffer, m_currentBufferSize);
	}

	inline void flushToStreamAndReset(std::ostream& stream) {
		flushToStream(stream);
		reset();
	}

	constexpr const byte_t* getBuffer() const {
		return m_buffer;
	}

	inline BufferReader createBufferReader() const {
		return BufferReader(m_buffer, m_bufferCapacity, m_currentBufferSize);
	}
};

template <uint64_t Size> class StaticBufferWriter : public BufferWriter {
public:
	StaticBufferWriter()
			: BufferWriter(nullptr, Size) {
		m_buffer = m_staticBuffer.data();
	}

private:
	std::array<byte_t, Size> m_staticBuffer;
};

class DynamicBufferWriter : public BufferWriter {
public:
	DynamicBufferWriter(const uint64_t bufferSize, const bool allowRealloc = false)
			: BufferWriter(nullptr, bufferSize)
			, m_allowRealloc(allowRealloc) {
		m_dynamicBuffer = std::make_unique<byte_t[]>(m_bufferCapacity);
		m_buffer = m_dynamicBuffer.get();
	}

	inline void writeBytes(const byte_t* bytes, const uint64_t numOfBytes) override {
		if (m_allowRealloc && m_currentBufferSize + numOfBytes > m_bufferCapacity) {
			realloc(numOfBytes);
		}

		BufferWriter::writeBytes(bytes, numOfBytes);
	}

	constexpr bool canWrite(const uint64_t numOfBytes) const override {
		return m_allowRealloc || BufferWriter::canWrite(numOfBytes);
	}

private:
	std::unique_ptr<byte_t[]> m_dynamicBuffer;
	const bool m_allowRealloc;

	inline void realloc(const uint64_t extra) {
		m_bufferCapacity = (m_bufferCapacity + extra) * 2ull;
		auto oldBuffer = std::move(m_dynamicBuffer);
		m_dynamicBuffer = std::make_unique<byte_t[]>(m_bufferCapacity);
		memcpy(m_dynamicBuffer.get(), oldBuffer.get(), m_currentBufferSize);
		m_buffer = m_dynamicBuffer.get();
	}
};

class BufferedFileHandler {
public:
	BufferedFileHandler() = delete;

	BufferedFileHandler(const std::string_view fileName)
			: m_fileName(fileName)
			, m_isOk(true) { }

	virtual ~BufferedFileHandler() { }

	constexpr std::string_view getName() const {
		return m_fileName;
	}

	constexpr bool isOk() const {
		return m_isOk;
	}

protected:
	static constexpr uint64_t BUFFER_SIZE = 4096;
	const std::string_view m_fileName;
	bool m_isOk;
};

class BufferedFileWriter : public BufferedFileHandler {
public:
	BufferedFileWriter(const std::string_view fileName)
			: BufferedFileHandler(fileName)
			, m_stream(fileName.data(), std::ios::out | std::ios::binary) {
		m_isOk = m_stream.good();
	}
	
	virtual ~BufferedFileWriter() {
		if (!flush()) {
			std::cout << "Failed writing to file: " << m_fileName << '\n';
		}

		m_stream.close();
	}

	template <typename T> inline bool write(const T& obj) {
		return writeBytes(reinterpret_cast<const byte_t*>(&obj), sizeof(T));
	}

	inline bool write(const std::string_view string) {
		return writeBytes(string.data(), string.size());
	}

	template <typename T> inline bool writeBytes(const T* bytes, const uint64_t numOfBytes) {
		return writeBytes(reinterpret_cast<const byte_t*>(bytes), numOfBytes);
	}
		
	inline bool writeBytes(const byte_t* bytes, const uint64_t numOfBytes) {
		if (!m_stream) {
			m_isOk = false;
			return false;
		}

		if (!m_bufferWriter.canWrite(numOfBytes)) {
			m_bufferWriter.flushToStreamAndReset(m_stream);
		}

		if (numOfBytes > BUFFER_SIZE) {
			m_stream.write(bytes, numOfBytes);
		}
		else {
			m_bufferWriter.writeBytes(bytes, numOfBytes);
		}

		m_isOk = m_stream.good();
		return m_stream.good();
	}

	inline bool flush() {
		m_bufferWriter.flushToStreamAndReset(m_stream);
		return m_stream.good();
	}

private:
	StaticBufferWriter<BUFFER_SIZE> m_bufferWriter;
	std::ofstream m_stream;
};

class BufferedFileReader : public BufferedFileHandler {
public:
	BufferedFileReader(const std::string_view fileName)
			: BufferedFileHandler(fileName)
			, m_stream(fileName.data(), std::ios::in | std::ios::binary) {
		m_isOk = m_stream.good();
		m_bufferReader.fill(m_stream);
	}

	template <typename T> inline bool read(T& objOut) {
		byte_t* objPtr = reinterpret_cast<byte_t*>(&objOut);
		return readBytes(objPtr, sizeof(objOut));
	}

	template <typename T> inline bool readBytes(T* buffer, const uint64_t numOfBytes) {
		return readBytes(reinterpret_cast<byte_t*>(buffer), numOfBytes);
	}

	inline bool readBytes(byte_t* buffer, const uint64_t numOfBytes) {
		if (!m_stream && m_bufferReader.getBytesAvailable() == 0ull) {
			m_isOk = false;
			return false;
		}

		const uint64_t bytesAvailableFromBuffer = m_bufferReader.getBytesAvailable();
		if (bytesAvailableFromBuffer < numOfBytes) {
			m_bufferReader.readAll(buffer);

			const uint64_t bytesToReadRemaining = numOfBytes - bytesAvailableFromBuffer;
			m_stream.read(buffer + bytesAvailableFromBuffer, bytesToReadRemaining);
			if (static_cast<uint64_t>(m_stream.gcount()) != bytesToReadRemaining) {
				m_isOk = false;
				return false;
			}

			m_bufferReader.resetAndFill(m_stream);
		}
		else {
			m_bufferReader.readBytesToBuffer(buffer, numOfBytes);
		}

		return true;
	}

private:
	std::ifstream m_stream;
	StaticBufferReader<BUFFER_SIZE> m_bufferReader;
};

}
