/*
 * Boost Pointer Pool - templated thread safe pointer pool
 *
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <memory>
#include <mutex>
#include <queue>

#include <boost/ptr_container/clone_allocator.hpp>
#include <boost/ptr_container/ptr_set.hpp>

namespace mke {
namespace memory {

/** \addtogroup mkecli
 *  @{
 */

/**
 * @brief Implements a thread-safe memory pool of pointers. Once a pointer
 * is added into the memory pool via the `add` method,
 * the `PointerPool` takes ownership of the pointer and
 * deallocates it in its destructor or via the `clear` method.
 *
 */
template <class T, class Compare = std::less<T>>
class BoostPointerPool
{
 public:
  ~BoostPointerPool()
  {
    clear();
  };

  /**
   * @brief Releases whatever data has been previously added into the pool,
   * allocates `no_items` of `T` objects and adds them to the pool.
   *
   * @param no_items p_no_items:...
   */
  void reserve(const int no_items)
  {
    std::unique_lock<std::mutex> mlock(queue_mutex_);

    pointers_.clear();
    std::queue<T*> empty_queue;
    std::swap(avail_queue_, empty_queue);

    for (int i = 0; i < no_items; i++)
      {
        T* ptr = new T();
        pointers_.insert(ptr);
        avail_queue_.push(ptr);
      }
  }

  std::size_t size()
  {
    return pointers_.size();
  }

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
      return false;

    std::unique_lock<std::mutex> mlock(queue_mutex_);

    // if (pointers_.find(*ptr) != pointers_.end())
    //   return false;

    pointers_.insert(ptr);
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

    //  if (pointers_.find(*ptr) == pointers_.end())
    //    return false;

    avail_queue_.push(ptr);
    ptr = nullptr;
    return true;
  }

  /**
   * @brief Releases add memory added via `add` or `reserve`.
   *
   */
  void clear(void)
  {
    std::unique_lock<std::mutex> mlock(queue_mutex_);

    std::queue<T*> empty_queue;
    std::swap(avail_queue_, empty_queue);
    //    pointers_.clear();
  }

 private:
  std::mutex queue_mutex_;
  std::queue<T*> avail_queue_;
  boost::ptr_set<T, Compare> pointers_;

  BoostPointerPool(const BoostPointerPool&); // noncopyable
  BoostPointerPool& operator=(const BoostPointerPool&); //
};


/** @}*/

} /* namespace memory */
} /* namespace mke */
