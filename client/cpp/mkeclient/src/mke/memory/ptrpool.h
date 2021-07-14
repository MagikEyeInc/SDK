/*
 * Pointer Pool - templated thread safe pointer pool
 *
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <memory>
#include <mutex>
#include <queue>

namespace mke {
namespace memory {

/** \addtogroup mkecli
 *  @{
 */

/**
 * @brief Implements a thread-safe pool of pointers.
 * Pool does not take unique ownership of the pointer
 * nor does it deallocated any memory.
 *
 */
template <class T>
class PointerPool
{
 public:
  PointerPool() {}

  ~PointerPool()
  {
    clear();
  };

  /**
   * @brief Returns the number of available pointers in the pool.
   *
   * @return std::size_t
   */
  std::size_t available()
  {
    std::unique_lock<std::mutex> mlock(queue_mutex_);

    return avail_queue_.size();
  }

  /**
   * @brief Adds a pointer to the pool. The pool will claim
   * ownership of the pointer and will the underlying memory
   * once the `clear` methods is called or in the pool's
   * destructor.
   *
   * @param ptr p_ptr:...
   * @return bool
   */
  bool add(T* ptr)
  {
    if (!ptr)
      {
        return false;
      }

    std::unique_lock<std::mutex> mlock(queue_mutex_);

    avail_queue_.push(ptr);
    return true;
  }

  /**
   * @brief Requests a pointer from the pool. The methods
   * will set the pointer via its parameters. The pointer
   * can be considered valid only if the methods returs `true`.
   *
   * @param ptr p_ptr:...
   * @return bool
   */
  bool request(T*& ptr)
  {
    std::unique_lock<std::mutex> mlock(queue_mutex_);

    if (avail_queue_.empty())
      {
        ptr = nullptr;
        return false;
      }

    ptr = avail_queue_.front();
    avail_queue_.pop();
    return true;
  }

  /**
   * @brief Releases a pointer previously borrowed via `request`
   * back into the pool.
   *
   * @param ptr p_ptr:...
   * @return bool
   */
  bool release(T*& ptr)
  {
    std::unique_lock<std::mutex> mlock(queue_mutex_);

    avail_queue_.push(ptr);
    ptr = nullptr;
    return true;
  }

  /**
   * @brief Releases add memory added via `add` or `reserve`.
   *
   */
  void clear()
  {
    std::unique_lock<std::mutex> mlock(queue_mutex_);

    std::queue<T*> empty_queue;
    std::swap(avail_queue_, empty_queue);
  }

 private:
  std::mutex queue_mutex_;
  std::queue<T*> avail_queue_;

  PointerPool(const PointerPool&); // noncopyable
  PointerPool& operator=(const PointerPool&); //
};


/** @}*/

} /* namespace memory */
} /* namespace mke */
