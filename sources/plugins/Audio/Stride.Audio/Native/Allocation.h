// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

#pragma once

// Allocation helpers for the native audio code. Stride's native build links the Linux library without a C++
// runtime, so the global operator new/delete would be left undefined in libstrideaudio.so and resolve only if
// the host executable itself depends on libstdc++, which the .NET 11 apphost does not. Everything here
// allocates through NativePath's malloc/free.

#include "../../../deps/NativePath/NativePath.h"
#include "../../../deps/NativePath/NativeMemory.h"

// Gives a class its own operator new/delete, so that `new T` and `delete p` use malloc/free instead of the
// global operators. Add it to every class that is created with new:
//
//   struct xnAudioDevice
//   {
//       XN_MALLOC_NEW_DELETE
//       ...
//   };
#define XN_MALLOC_NEW_DELETE \
	static void* operator new(size_t size) { return malloc(size); } \
	static void operator delete(void* ptr) { free(ptr); }

// Allocator for the tinystl containers; the default tinystl::allocator calls the global operator new/delete.
// Include this header before any TINYSTL header so that TINYSTL_ALLOCATOR is picked up.
struct xnAllocator
{
	static void* static_allocate(size_t bytes) { return malloc(bytes); }
	static void static_deallocate(void* ptr, size_t /*bytes*/) { free(ptr); }
};
#define TINYSTL_ALLOCATOR xnAllocator
