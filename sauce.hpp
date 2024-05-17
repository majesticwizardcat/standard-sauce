#pragma once

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cassert>
#include <fstream>
#include <cstring>
#include <cstdint>

namespace consts {

static constexpr float ERROR = 0.0001f;
static constexpr float CLOSE_ERROR = 0.5f;
static constexpr float PI = 3.14159f;
static constexpr float TWO_PI = 2.0f * PI;
static constexpr float PI_OVER_TWO = PI * 0.5f;

}

typedef char byte_t;

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
	inline constexpr auto begin() const { return m_array.begin(); }
	inline constexpr auto end() const { return m_array.begin() + m_size; }
	inline constexpr bool empty() const { return m_size == 0; }
	inline constexpr const T* data() const { return m_array.data(); }
	inline constexpr const T& operator[](const uint64_t pos) const { assert(pos < m_size); return m_array[pos]; }

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

inline static constexpr bool are_equal(const float f0, const float f1) {
	return std::abs(f0 - f1) < consts::ERROR;
}

inline static constexpr bool are_not_equal(const float f0, const float f1) {
	return std::abs(f0 - f1) > consts::ERROR;
}

inline static constexpr bool is_zero(const float f) {
	return are_equal(f, 0.0f);
}

inline static constexpr bool are_close(const float f0, const float f1) {
	return std::abs(f0 - f1) < consts::CLOSE_ERROR;
}

inline static constexpr bool are_not_close(const float f0, const float f1) {
	return std::abs(f0 - f1) > consts::CLOSE_ERROR;
}

inline static float fast_inv_sqrt(float f) {
	constexpr float threehalves = 1.5f;
	const float x2 = f * 0.5f;

	long long i = *reinterpret_cast<long long*>(&f);
	i = 0x5f3759df - (i >> 1);
	f = *reinterpret_cast<float*>(&i);

	f = f * (threehalves - (x2 * f * f));
	f = f * (threehalves - (x2 * f * f));
	return f;
}

inline static constexpr float fast_sqrt(const float f) {
	return 1.0f / fast_inv_sqrt(f);
}

//////////////////////////// TOKENIZER //////////////////////////// 

inline static constexpr std::vector<std::string_view> tokenize(const char* str, const uint64_t size, const char delim = ' ') {
	static constexpr uint64_t minTokens = 8;

	std::vector<std::string_view> tokens;
	tokens.reserve(minTokens);

	uint64_t curWordStart = 0;
	for (uint64_t i = 0; i < size; ++i) {
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

inline static constexpr std::vector<std::string_view> tokenize(const std::string& str, const char delim = ' ') {
	return sauce::tokenize(str.c_str(), str.size(), delim);
}

inline static constexpr std::vector<std::string_view> tokenize(const std::string_view str, const char delim = ' ') {
	return sauce::tokenize(str.data(), str.size(), delim);
}

//////////////////////////// BYTE IO //////////////////////////// 

class Buffer {
public:
	Buffer(void* buffer, const uint64_t capacity)
			: m_buffer(reinterpret_cast<byte_t*>(buffer))
			, m_bufferCapacity(capacity)
			, m_currentBufferSize(0) { }

	virtual inline constexpr void reset() {
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

	inline constexpr uint64_t getBytesAvailable() const {
		return m_bufferReadEnd - m_currentBufferSize;
	}

	inline constexpr void reset() override {
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

	template <typename T> inline constexpr bool canWrite(const T& obj) const {
		return canWrite(sizeof(obj));
	}

	inline virtual constexpr bool canWrite(const uint64_t numOfBytes) const {
		return m_currentBufferSize + numOfBytes <= m_bufferCapacity;
	}

	inline constexpr uint64_t bytesWritten() const {
		return m_currentBufferSize;
	}

	inline constexpr uint64_t availableSpace() const {
		return m_bufferCapacity - m_currentBufferSize;
	}

	inline void flushToStream(std::ostream& stream) const {
		stream.write(m_buffer, m_currentBufferSize);
	}

	inline void flushToStreamAndReset(std::ostream& stream) {
		flushToStream(stream);
		reset();
	}

	inline constexpr const byte_t* getBuffer() const {
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

	inline constexpr bool canWrite(const uint64_t numOfBytes) const override {
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

	inline constexpr std::string_view getName() const {
		return m_fileName;
	}

	inline constexpr bool isOk() const {
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
			, m_stream(fileName, std::ios::out | std::ios::binary) {
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
			, m_stream(fileName, std::ios::in | std::ios::binary) {
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
