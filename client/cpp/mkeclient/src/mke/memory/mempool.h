
/* -------------------------------------------------------------------------- */

#pragma once

/* -------------------------------------------------------------------------- */

#include <vector>
#include <queue>

/* -------------------------------------------------------------------------- */

namespace mke {
namespace memory {

/* -------------------------------------------------------------------------- */

/**
 * @brief MemPool - a thread-safe pool of pointers.
 *
 */
template <typename Type>
class MemPool : protected std::vector<Type>

{
protected:
  std::queue<Type *>            free_items_;    // queue of available items
  
public:
  using std::vector<Type>::size;
  
  // contructor
  MemPool(size_t pool_size):
    std::vector<Type>(pool_size)
  {
    for(typename std::vector<Type>::iterator ptr = this->begin(); ptr != this->end(); ++ptr)
      free_items_.push(&*ptr);
  };
  
  // destructor
  virtual ~MemPool() {};
  
  // get next item that is available
  Type * getNextAvailable() 
  {
    if(free_items_.empty())
      return nullptr;
    
    Type * ret = free_items_.front();
    free_items_.pop();
    
    return ret;
  };
  
  // return back item
  void returnBack(Type * item)
  {
    free_items_.push(item);
  }
  
};

/* -------------------------------------------------------------------------- */

} // end of mke::memory
} // end of mke

/* -------------------------------------------------------------------------- */
