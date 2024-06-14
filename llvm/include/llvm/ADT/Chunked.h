//===- Chunked.h - For chunked views/refs -----------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_CHUNKED_H
#define LLVM_ADT_CHUNKED_H

#include "llvm/ADT/ArrayRef.h"

namespace llvm {
  namespace chunked {
  template<typename T>
  class ArrayIterator {
  public:
    using value_type = ArrayRef<T>;
    using reference = ArrayRef<T>;
    using pointer = void;
    using size_type = size_t;
    using difference_type = ptrdiff_t;
    using iterator_category = std::random_access_iterator_tag;
  private:
    const T *Data;
    size_type ElPerChunk;
  public:
    constexpr const T *data() const { return Data; }
    constexpr size_type chunk_size() const { return ElPerChunk; }

    constexpr ArrayIterator(const T *Data, size_t ChunkSize)
      : Data(Data), ElPerChunk(ChunkSize)
    {}

    constexpr bool operator==(const ArrayIterator &It) const {
      assert(ElPerChunk == It.ElPerChunk && "Invalid ArrayIterator operation");
      return Data == It.Data;
    }
    constexpr bool operator!=(const ArrayIterator &It) const {
      assert(ElPerChunk == It.ElPerChunk && "Invalid ArrayIterator operation");
      return Data != It.Data;
    }
    constexpr bool operator<=(const ArrayIterator &It) const {
      assert(ElPerChunk == It.ElPerChunk && "Invalid ArrayIterator operation");
      return Data <= It.Data;
    }
    constexpr bool operator<(const ArrayIterator &It) const {
      assert(ElPerChunk == It.ElPerChunk && "Invalid ArrayIterator operation");
      return Data < It.Data;
    }
    constexpr bool operator>=(const ArrayIterator &It) const {
      assert(ElPerChunk == It.ElPerChunk && "Invalid ArrayIterator operation");
      return Data >= It.Data;
    }
    constexpr bool operator>(const ArrayIterator &It) const {
      assert(ElPerChunk == It.ElPerChunk && "Invalid ArrayIterator operation");
      return Data > It.Data;
    }

    constexpr ArrayIterator& operator++() {
      Data += ElPerChunk;
      return *this;
    }

    constexpr ArrayIterator& operator--() {
      Data -= ElPerChunk;
      return *this;
    }

    constexpr ArrayIterator operator+(size_t Index) const {
      return ArrayIterator(Data + Index * ElPerChunk, ElPerChunk);
    }

    constexpr ArrayIterator operator-(size_t Index) const {
      return ArrayIterator(Data - Index * ElPerChunk, ElPerChunk);
    }

    constexpr difference_type operator-(const ArrayIterator &It) const {
      assert(ElPerChunk == It.ElPerChunk && "Invalid ArrayIterator operation");
      difference_type D = Data - It.Data;
      assert(D % ElPerChunk == 0);
      return D / ElPerChunk;
    }

    constexpr ArrayIterator& operator+=(size_t Index) {
      Data += Index * ElPerChunk;
      return *this;
    }

    constexpr ArrayIterator& operator-=(size_t Index) {
      Data -= Index * ElPerChunk;
      return *this;
    }

    constexpr reference operator*() const {
      return ArrayRef(Data, ElPerChunk);
    }
  };

  template<typename T>
  class MutableArrayIterator : public ArrayIterator<T> {
  public:
    using value_type = MutableArrayRef<T>;
    using reference = MutableArrayRef<T>;
    using const_reference = ArrayRef<T>;
    using pointer = void;
    using size_type = size_t;
    using difference_type = ptrdiff_t;
    using iterator_category = std::random_access_iterator_tag;

    constexpr T *data() const { return const_cast<T *>(this->ArrayIterator<T>::data()); }

    constexpr MutableArrayIterator(const T *Data, size_t Chunksize)
      : ArrayIterator<T>(Data, Chunksize)
    {}

    constexpr MutableArrayIterator operator+(size_t Index) const {
      return MutableArrayIterator(this->data() + Index * this->chunk_size(), this->chunk_size());
    }

    constexpr MutableArrayIterator operator-(size_t Index) const {
      return MutableArrayIterator(this->data() - Index * this->chunk_size(), this->chunk_size());
    }

    constexpr difference_type operator-(const MutableArrayIterator &It) const {
      return this->ArrayIterator<T>::operator-(static_cast<const ArrayIterator<T> &>(It));
    }

    constexpr MutableArrayIterator &operator++() {
      this->ArrayIterator<T>::operator++(); 
      return *this;
    }

    constexpr MutableArrayIterator &operator--() {
      this->ArrayIterator<T>::operator--(); 
      return *this;
    }

    constexpr MutableArrayIterator &operator+=(size_t Index) {
      this->ArrayIterator<T>::operator+=(Index); 
      return *this;
    }

    constexpr MutableArrayIterator &operator-=(size_t Index) {
      this->ArrayIterator<T>::operator-=(Index); 
      return *this;
    }

    constexpr reference operator*() const {
      return MutableArrayRef(this->data(), this->chunk_size());
    }
  };
  } // end of namespace chunked

  /// ChunkedArrayRef - Represents an ArrayRef which has elements
  /// grouped into fixed size repeating chunks
  ///
  /// Lifetime semantics are the same as an ArrayRef
  /// Every chunk must be the full size, if the range
  /// is not a multiple of the number of elements per chunk
  /// then the final chunk (containing the odd number of elements)
  /// is not included
  template<typename T>
  class [[nodiscard]] ChunkedArrayRef {
  public:
    using value_type = ArrayRef<T>;
    using reference = ArrayRef<T>;
    using const_reference = ArrayRef<T>;
    using pointer = void;
    using iterator = chunked::ArrayIterator<T>;
    using const_iterator = chunked::ArrayIterator<T>;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    using size_type = size_t;
    using difference_type = ptrdiff_t;

  private:
    /// The start of the array, in an external buffer.
    const T *Data = nullptr;

    /// The number of chunks.
    size_type Chunks = 0;

    /// The number of elements per chunk.
    size_type ElPerChunk = 0;

  public:
    /// @name Constructors
    /// @{

    /// Construct an empty ChunkedArrayRef.
    /*implicit*/ ChunkedArrayRef() = default;

    /// Construct an empty ChunkedArrayRef from std::nullopt.
    /*implicit*/ ChunkedArrayRef(std::nullopt_t) {}

    /// Construct an ChunkedArrayRef from a pointer, a length and a chunk size.
    constexpr /*implicit*/ ChunkedArrayRef(const T *data, size_t length, size_t chunksize)
        : Data(data), Chunks(length / chunksize), ElPerChunk(chunksize) {}


    /// Construct a MutableArrayRef from a range.
    ChunkedArrayRef(const chunked::ArrayIterator<T> &Begin,
                    const chunked::ArrayIterator<T> &End)
      : ChunkedArrayRef(Begin.data(), 
                        (Begin - End) * Begin.chunk_size(),
                        Begin.chunk_size()) {
      assert(Begin.chunk_size() == End.chunk_size() && "Invalid iterator pair");
    }

    /// Construct an ArrayRef<const T*> from ArrayRef<T*>. This uses SFINAE to
    /// ensure that only ArrayRefs of pointers can be converted.
    constexpr ChunkedArrayRef(const ArrayRef<T> &A, size_t chunksize)
        : ChunkedArrayRef(A.data(), A.size(), chunksize) {}

    /// Construct an ChunkedArrayRef from a SmallVector. This is templated in order to
    /// avoid instantiating SmallVectorTemplateCommon<T> whenever we
    /// copy-construct an ChunkedArrayRef.
    template<typename U>
    /*implicit*/ ChunkedArrayRef(const SmallVectorTemplateCommon<T, U> &Vec, size_t chunksize)
      : ChunkedArrayRef(Vec.data(), Vec.size(), chunksize) {
    }

    /// Construct an ChunkedArrayRef from a std::vector.
    template<typename A>
    /*implicit*/ ChunkedArrayRef(const std::vector<T, A> &Vec, size_t chunksize)
      : ChunkedArrayRef(Vec.data(), Vec.size(), chunksize) {}

    /// Construct an ChunkedArrayRef from a std::array
    template <size_t N>
    /*implicit*/ constexpr ChunkedArrayRef(const std::array<T, N> &Arr, size_t chunksize)
        : ChunkedArrayRef(Arr.data(), N, chunksize) {}

    /// Construct an ChunkedArrayRef from a C array.
    template <size_t N>
    /*implicit*/ constexpr ChunkedArrayRef(const T (&Arr)[N], size_t chunksize)
      : ChunkedArrayRef(Arr, N, chunksize) {}

    /// Construct an ArrayRef<const T*> from ArrayRef<T*>. This uses SFINAE to
    /// ensure that only ArrayRefs of pointers can be converted.
    template <typename U>
    ChunkedArrayRef(const ArrayRef<U *> &A, size_t chunksize,
             std::enable_if_t<std::is_convertible<U *const *, T const *>::value>
                 * = nullptr)
        : ChunkedArrayRef(A.data(), A.size(), chunksize) {}

    /// Construct an ArrayRef<const T*> from a SmallVector<T*>. This is
    /// templated in order to avoid instantiating SmallVectorTemplateCommon<T>
    /// whenever we copy-construct an ArrayRef.
    template <typename U, typename DummyT>
    /*implicit*/ ChunkedArrayRef(
        const SmallVectorTemplateCommon<U *, DummyT> &Vec, size_t chunksize,
        std::enable_if_t<std::is_convertible<U *const *, T const *>::value> * =
            nullptr)
        : ChunkedArrayRef(Vec.data(), Vec.size(), chunksize) {}

    /// Construct an ArrayRef<const T*> from std::vector<T*>. This uses SFINAE
    /// to ensure that only vectors of pointers can be converted.
    template <typename U, typename A>
    ChunkedArrayRef(const std::vector<U *, A> &Vec, size_t chunksize,
             std::enable_if_t<std::is_convertible<U *const *, T const *>::value>
                 * = nullptr)
        : ChunkedArrayRef(Vec.data(), Vec.size(), chunksize) {}

    /// @}
    /// @name Simple Operations
    /// @{

    iterator begin() const { return chunked::ArrayIterator(Data, ElPerChunk); }
    iterator end() const { return chunked::ArrayIterator(Data + Chunks * ElPerChunk, ElPerChunk); }

    reverse_iterator rbegin() const { return reverse_iterator(end()); }
    reverse_iterator rend() const { return reverse_iterator(begin()); }

    /// empty - Check if the array is empty.
    bool empty() const { return Chunks == 0; }

    const T *data() const { return Data; }

    /// size - Get the array size.
    size_t size() const { return Chunks; }
    size_t chunk_size() const { return ElPerChunk; }

    /// front - Get the first element.
    const_reference front() const {
      assert(!empty());
      return operator[](0);
    }

    /// back - Get the last element.
    const_reference back() const {
      assert(!empty());
      return operator[](Chunks-1);
    }

    /// equals - Check for element-wise equality.
    bool equals(ChunkedArrayRef RHS) const {
      if (ElPerChunk != RHS.ElPerChunk || Chunks != RHS.Chunks)
        return false;
      return std::equal(begin(), end(), RHS.begin());
    }

    /// slice(n, m) - Chop off the first N elements of the array, and keep M
    /// elements in the array.
    ChunkedArrayRef<T> slice(size_t N, size_t M) const {
      assert(N+M <= size() && "Invalid specifier");
      return ChunkedArrayRef<T>(data()+N*ElPerChunk, M * ElPerChunk, ElPerChunk);
    }

    /// slice(n) - Chop off the first N elements of the array.
    ChunkedArrayRef<T> slice(size_t N) const { return slice(N, size() - N); }

    /// Drop the first \p N elements of the array.
    ChunkedArrayRef<T> drop_front(size_t N = 1) const {
      assert(size() >= N && "Dropping more elements than exist");
      return slice(N, size() - N);
    }

    /// Drop the last \p N elements of the array.
    ChunkedArrayRef<T> drop_back(size_t N = 1) const {
      assert(size() >= N && "Dropping more elements than exist");
      return slice(0, size() - N);
    }

    /// Return a copy of *this with the first N elements satisfying the
    /// given predicate removed.
    template <class PredicateT> ChunkedArrayRef<T> drop_while(PredicateT Pred) const {
      return ChunkedArrayRef<T>(find_if_not(*this, Pred), end());
    }

    /// Return a copy of *this with the first N elements not satisfying
    /// the given predicate removed.
    template <class PredicateT> ChunkedArrayRef<T> drop_until(PredicateT Pred) const {
      return ChunkedArrayRef<T>(find_if(*this, Pred), end());
    }

    /// Return a copy of *this with only the first \p N elements.
    ChunkedArrayRef<T> take_front(size_t N = 1) const {
      if (N >= size())
        return *this;
      return drop_back(size() - N);
    }

    /// Return a copy of *this with only the last \p N elements.
    ChunkedArrayRef<T> take_back(size_t N = 1) const {
      if (N >= size())
        return *this;
      return drop_front(size() - N);
    }

    /// Return the first N elements of this Array that satisfy the given
    /// predicate.
    template <class PredicateT> ChunkedArrayRef<T> take_while(PredicateT Pred) const {
      return ChunkedArrayRef<T>(begin(), find_if_not(*this, Pred));
    }

    /// Return the first N elements of this Array that don't satisfy the
    /// given predicate.
    template <class PredicateT> ChunkedArrayRef<T> take_until(PredicateT Pred) const {
      return ChunkedArrayRef<T>(begin(), find_if(*this, Pred));
    }

    /// @}
    /// @name Operator Overloads
    /// @{
    const_reference operator[](size_t Index) const {
      assert(Index < Chunks && "Invalid index!");
      return ArrayRef(Data + Index * ElPerChunk, ElPerChunk);
    }

    /// Disallow accidental assignment from a temporary.
    ///
    /// The declaration here is extra complicated so that "arrayRef = {}"
    /// continues to select the move assignment operator.
    template <typename U>
    std::enable_if_t<std::is_same<U, T>::value, ArrayRef<T>> &
    operator=(U &&Temporary) = delete;

    /// Disallow accidental assignment from a temporary.
    ///
    /// The declaration here is extra complicated so that "arrayRef = {}"
    /// continues to select the move assignment operator.
    template <typename U>
    std::enable_if_t<std::is_same<U, T>::value, ArrayRef<T>> &
    operator=(std::initializer_list<U>) = delete;

    /// @}
  };

  template<typename T>
  class [[nodiscard]] ChunkedMutableArrayRef : public ChunkedArrayRef<T> {
  public:
    using value_type = ArrayRef<T>;
    using reference = MutableArrayRef<T>;
    using pointer = void;
    using iterator = chunked::MutableArrayIterator<T>;
    using const_iterator = chunked::ArrayIterator<T>;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    using size_type = size_t;
    using difference_type = ptrdiff_t;

  private:
    constexpr T *data() const { return const_cast<T *>(this->ChunkedArrayRef<T>::data()); }
  public:
    /// @name Constructors
    /// @{

    /// Construct an empty ChunkedMutableArrayRef.
    /*implicit*/ constexpr ChunkedMutableArrayRef() = default;

    /// Construct an empty ChunkedMutableArrayRef from std::nullopt.
    /*implicit*/ constexpr ChunkedMutableArrayRef(std::nullopt_t) : ChunkedArrayRef<T>(std::nullopt) {}

    /// Construct an ChunkedArrayRef from a pointer, a length and a chunk size.
    constexpr /*implicit*/ ChunkedMutableArrayRef(T *data, size_t length, size_t chunksize)
        : ChunkedArrayRef<T>(data, length, chunksize) {}


    /// Construct a MutableArrayRef from a range.
    constexpr ChunkedMutableArrayRef(const chunked::MutableArrayIterator<T> &Begin,
                                     const chunked::MutableArrayIterator<T> &End)
      : ChunkedMutableArrayRef(Begin.data(), 
                               (Begin - End) * Begin.chunk_size(),
                               Begin.chunk_size()) {
      assert(Begin.chunk_size() == End.chunk_size() && "Invalid iterator pair");
    }

    /// Construct an ArrayRef<const T*> from ArrayRef<T*>. This uses SFINAE to
    /// ensure that only ArrayRefs of pointers can be converted.
    constexpr ChunkedMutableArrayRef(const MutableArrayRef<T> &A, size_t chunksize)
        : ChunkedMutableArrayRef(A.data(), A.size(), chunksize) {}

    /// Construct an ChunkedArrayRef from a SmallVector. This is templated in order to
    /// avoid instantiating SmallVectorTemplateCommon<T> whenever we
    /// copy-construct an ChunkedArrayRef.
    template<typename U>
    /*implicit*/ ChunkedMutableArrayRef(SmallVectorTemplateCommon<T, U> &Vec, size_t chunksize)
      : ChunkedMutableArrayRef(Vec.data(), Vec.size(), chunksize) {
    }

    /// Construct an ChunkedArrayRef from a std::vector.
    template<typename A>
    /*implicit*/ ChunkedMutableArrayRef(std::vector<T, A> &Vec, size_t chunksize)
      : ChunkedMutableArrayRef(Vec.data(), Vec.size(), chunksize) {}

    /// Construct an ChunkedArrayRef from a std::array
    template <size_t N>
    /*implicit*/ constexpr ChunkedMutableArrayRef(std::array<T, N> &Arr, size_t chunksize)
        : ChunkedMutableArrayRef(Arr.data(), N, chunksize) {}

    /// Construct an ChunkedArrayRef from a C array.
    template <size_t N>
    /*implicit*/ constexpr ChunkedMutableArrayRef(T (&Arr)[N], size_t chunksize)
      : ChunkedMutableArrayRef(Arr, N, chunksize) {}

    /// Construct an ArrayRef<const T*> from ArrayRef<T*>. This uses SFINAE to
    /// ensure that only ArrayRefs of pointers can be converted.
    template <typename U>
    ChunkedMutableArrayRef(ArrayRef<U *> &A, size_t chunksize,
             std::enable_if_t<std::is_convertible<U *const *, T const *>::value>
                 * = nullptr)
        : ChunkedMutableArrayRef(A.data(), A.size(), chunksize) {}

    /// Construct an ArrayRef<const T*> from a SmallVector<T*>. This is
    /// templated in order to avoid instantiating SmallVectorTemplateCommon<T>
    /// whenever we copy-construct an ArrayRef.
    template <typename U, typename DummyT>
    /*implicit*/ ChunkedMutableArrayRef(
        SmallVectorTemplateCommon<U *, DummyT> &Vec, size_t chunksize,
        std::enable_if_t<std::is_convertible<U *const *, T const *>::value> * =
            nullptr)
        : ChunkedMutableArrayRef(Vec.data(), Vec.size(), chunksize) {}

    /// Construct an ArrayRef<const T*> from std::vector<T*>. This uses SFINAE
    /// to ensure that only vectors of pointers can be converted.
    template <typename U, typename A>
    ChunkedMutableArrayRef(std::vector<U *, A> &Vec, size_t chunksize,
             std::enable_if_t<std::is_convertible<U *const *, T const *>::value>
                 * = nullptr)
        : ChunkedMutableArrayRef(Vec.data(), Vec.size(), chunksize) {}

    /// @}
    /// @name Simple Operations
    /// @{

    iterator begin() const { return chunked::MutableArrayIterator(this->data(), this->chunk_size()); }
    iterator end() const { return chunked::MutableArrayIterator(this->data() + this->size() * this->chunk_size(), this->chunk_size()); }
  
    reverse_iterator rbegin() const { return reverse_iterator(this->end()); }
    reverse_iterator rend() const { return reverse_iterator(this->begin()); }

    /// front - Get the first element.
    reference front() const {
      assert(!this->empty());
      return operator[](0);
    }

    /// back - Get the last element.
    reference back() const {
      assert(!this->empty());
      return operator[](this->size()-1);
    }

    /// slice(n, m) - Chop off the first N elements of the array, and keep M
    /// elements in the array.
    ChunkedMutableArrayRef<T> slice(size_t N, size_t M) const {
      assert(N+M <= this->size() && "Invalid specifier");
      const size_t ElPerChunk = this->chunk_size();
      return ChunkedMutableArrayRef<T>(this->data() + N * ElPerChunk, M * ElPerChunk, ElPerChunk);
    }

    /// slice(n) - Chop off the first N elements of the array.
    ChunkedMutableArrayRef<T> slice(size_t N) const { return this->slice(N, this->size() - N); }

    /// Drop the first \p N elements of the array.
    ChunkedMutableArrayRef<T> drop_front(size_t N = 1) const {
      assert(this->size() >= N && "Dropping more elements than exist");
      return this->slice(N, this->size() - N);
    }

    /// Drop the last \p N elements of the array.
    ChunkedMutableArrayRef<T> drop_back(size_t N = 1) const {
      assert(this->size() >= N && "Dropping more elements than exist");
      return this->slice(0, this->size() - N);
    }

    /// Return a copy of *this with the first N elements satisfying the
    /// given predicate removed.
    template <class PredicateT> ChunkedMutableArrayRef<T> drop_while(PredicateT Pred) const {
      return ChunkedMutableArrayRef<T>(find_if_not(*this, Pred), this->end());
    }

    /// Return a copy of *this with the first N elements not satisfying
    /// the given predicate removed.
    template <class PredicateT> ChunkedMutableArrayRef<T> drop_until(PredicateT Pred) const {
      return ChunkedMutableArrayRef<T>(find_if(*this, Pred), this->end());
    }

    /// Return a copy of *this with only the first \p N elements.
    ChunkedMutableArrayRef<T> take_front(size_t N = 1) const {
      if (N >= this->size())
        return *this;
      return drop_back(this->size() - N);
    }

    /// Return a copy of *this with only the last \p N elements.
    ChunkedMutableArrayRef<T> take_back(size_t N = 1) const {
      if (N >= this->size())
        return *this;
      return drop_front(this->size() - N);
    }

    /// Return the first N elements of this Array that satisfy the given
    /// predicate.
    template <class PredicateT> ChunkedMutableArrayRef<T> take_while(PredicateT Pred) const {
      return ChunkedMutableArrayRef<T>(this->begin(), find_if_not(*this, Pred));
    }

    /// Return the first N elements of this Array that don't satisfy the
    /// given predicate.
    template <class PredicateT> ChunkedMutableArrayRef<T> take_until(PredicateT Pred) const {
      return ChunkedMutableArrayRef<T>(this->begin(), find_if(*this, Pred));
    }

    /// @}
    /// @name Operator Overloads
    /// @{
    reference operator[](size_t Index) {
      assert(Index < this->size() && "Invalid index!");
      return MutableArrayRef(this->data() + Index * this->chunk_size(), this->chunk_size());
    }

    /// @}
  };


} // end namespace llvm

#endif // LLVM_ADT_CHUNKED_H
