#ifndef __FIFO_H
#define __FIFO_H

//size: 4,8,16,32...256
struct fifo {
    volatile char *buf;
	volatile unsigned short size;
    volatile unsigned short tail;
    volatile unsigned short head;
};

struct fifo16 {
    volatile short *buf;
	volatile unsigned short size;
    volatile unsigned short tail;
    volatile unsigned short head;
};

#define FIFO_STATIC(_fifo_,_buffer_,_size_) static char _buffer_[_size_]; \
static struct fifo _fifo_ = {_buffer_, _size_, 0, 0};

#define FIFO16_STATIC(_fifo_,_buffer_,_size_) static short _buffer_[_size_]; \
static struct fifo16 _fifo_ = {_buffer_, _size_, 0, 0};

#define FIFO_MAKE(_fifo_,_buffer_,_size_) do { _fifo_.buf = _buffer_; _fifo_.size = _size_; } while (0)

#define FIFO_IS_FULL(fifo)   (((fifo)->head - (fifo)->tail) == (fifo)->size)
#define FIFO_IS_EMPTY(fifo)  ((fifo)->tail == (fifo)->head)
#define FIFO_COUNT(fifo)     ((fifo)->head - (fifo)->tail)
#define FIFO_SIZE(fifo)      ((fifo)->size)
#define FIFO_SPACE(fifo)     (FIFO_SIZE(fifo) - FIFO_COUNT(fifo))

#define FIFO_PUSH(fifo, byte) \
  {\
    (fifo)->buf[(fifo)->head & ((fifo)->size - 1)] = byte;\
    (fifo)->head = ((fifo)->head + 1) & ((fifo)->size - 1);\
  }

#define FIFO_FRONT(fifo) ((fifo)->buf[(fifo)->tail & ((fifo)->size - 1)])

#define FIFO_POP(fifo)   \
  {\
      (fifo)->tail = ((fifo)->tail + 1) & ((fifo)->size - 1); \
  }

#define FIFO_FLUSH(fifo)   \
  {\
    (fifo)->tail = 0;\
    (fifo)->head = 0;\
  }

#define FIFO_FIRST_PTR(fifo) (&(fifo)->buf[(fifo)->tail])
#define FIFO_FIRST(fifo) ((fifo)->buf[(fifo)->tail])
#define FIFO_LAST_PTR(fifo) (&(fifo)->buf[(fifo)->head])
#define FIFO_LAST(fifo) ((fifo)->buf[((fifo)->head - 1) & ((fifo)->size - 1)])
#define FIFO_APPEND(fifo,sz) ((fifo)->head = ((fifo)->head + (sz)) & ((fifo)->size - 1))
#define FIFO_EXTRACT(fifo,sz) ((fifo)->tail = ((fifo)->tail + (sz)) & ((fifo)->size - 1))
#define FIFO_REMOVE(fifo,sz) ((fifo)->head = ((fifo)->head - (sz)) & ((fifo)->size - 1))
#define FIFO_AT(fifo,index) ((fifo)->buf[(index)])

#endif /* __FIFO_H */
