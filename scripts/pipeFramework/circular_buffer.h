#include <stdlib.h>
#include <cstddef>
#include <cstdlib>

class FloatCircularBuffer {
  float *m_buffer;
  int m_head;
  int m_tail;
  float m_sum;
  float m_average;
  int m_count;
  const int m_size;
  const int m_average_len;
  const float m_threshold;
 
  int next(int current) {
    return (current + 1) % m_size;
  }
 
public:
 
  FloatCircularBuffer(const int size, const int average_len, const float threshold) : m_average_len(average_len), m_size(size), m_threshold(threshold), m_head(0), m_tail(0) {
    m_count = 0;
    m_sum = 0;
    m_average = 0;
    m_buffer = new float[size];
  }
 
  virtual ~FloatCircularBuffer() {
    delete [] m_buffer;
  }
 
  bool push(float object) {
    int head = m_head;
    int nextHead = next(head);
    if (head == m_tail && m_count == m_size) {
      m_tail = next(m_tail);
      m_count--;
    }
    m_buffer[head] = object;
    m_head = nextHead;

    m_sum += object;
    m_count++;
    if (m_count > m_average_len) {
      int subtract = (head-m_average_len > 0) ?
          (head-m_average_len) : (head-m_average_len + m_size);
      m_sum -= m_buffer[subtract];
      m_average = m_sum / m_average_len;
      printf("push head %i tail % i count %i\n", head, m_tail, m_count);
      return (m_average > m_threshold);
    }
    printf("push head %i tail % i count %i\n", head, m_tail, m_count);
    return false;
  }
 
  bool pop(float &object) {
    int tail = m_tail;
    if (tail == m_head && m_count == 0) {
      return false;
    }
 
    object = m_buffer[tail];
    m_tail = next(tail);
    m_count--;
    return true;
  }
};