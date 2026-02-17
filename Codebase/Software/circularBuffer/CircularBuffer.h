// CircularBuffer.h
#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

template <typename T>
class CircularBuffer
{
public:
    // Constructor
    explicit CircularBuffer(size_t capacity)
        : capacity_(capacity), head_(0), tail_(0), count_(0)
    {
        if (capacity_ == 0)
        {
            throw std::invalid_argument("CircularBuffer capacity cannot be zero.");
        }
        buffer_.resize(capacity_);
    }

    // Default constructor
    CircularBuffer()
        : CircularBuffer(1)
    {
        clear();
    }

    // Standard copy and move constructors/assignments
    CircularBuffer(const CircularBuffer &other) = default;
    CircularBuffer &operator=(const CircularBuffer &other) = default;
    CircularBuffer(CircularBuffer &&other) noexcept = default;
    CircularBuffer &operator=(CircularBuffer &&other) noexcept = default;

    // Add an element to the back
    void push(const T &item)
    {
        buffer_[tail_] = item;
        tail_ = (tail_ + 1) % capacity_;
        if (count_ < capacity_)
        {
            count_++;
        }
        else
        {
            // If the buffer was full, head moves with tail
            head_ = tail_;
        }
    }

    // Move an element to the back
    void push(T &&item)
    {
        buffer_[tail_] = std::move(item);
        tail_ = (tail_ + 1) % capacity_;
        if (count_ < capacity_)
        {
            count_++;
        }
        else
        {
            head_ = tail_;
        }
    }

    size_t size() const { return count_; }
    size_t capacity() const { return capacity_; }
    bool empty() const { return count_ == 0; }
    void clear()
    {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

    void reconfigure_capacity(size_t new_capacity)
    {
        if (new_capacity == 0)
        {
            throw std::invalid_argument("CircularBuffer new capacity cannot be zero.");
        }
        if (new_capacity != capacity_)
        {
            buffer_.clear();
            buffer_.resize(new_capacity);
            capacity_ = new_capacity;
            clear();
        }
        else
        {
            clear();
        }
    }

    // This lets us read elements in order without making a copy.
    const T &operator[](size_t index) const
    {
        if (index >= count_)
        {
            throw std::out_of_range("CircularBuffer index out of range.");
        }
        // This calculation translates a logical index (0 to size-1)
        // into the physical index in the underlying vector.
        return buffer_[(head_ + index) % capacity_];
    }

    // Convert buffer to a vector. This is still needed for sending the
    // final JIT packet, but we've eliminated it from the hot loop.
    std::vector<T> toVector() const
    {
        std::vector<T> result;
        if (empty())
        {
            return result;
        }
        result.reserve(count_);
        if (head_ < tail_)
        {
            // Elements are in a single contiguous block
            result.assign(buffer_.begin() + head_, buffer_.begin() + tail_);
        }
        else
        {
            // Elements wrap around the end of the vector
            result.assign(buffer_.begin() + head_, buffer_.end());
            result.insert(result.end(), buffer_.begin(), buffer_.begin() + tail_);
        }
        return result;
    }

private:
    std::vector<T> buffer_;
    size_t capacity_;
    size_t head_;
    size_t tail_;
    size_t count_;
};

#endif // CIRCULAR_BUFFER_H
