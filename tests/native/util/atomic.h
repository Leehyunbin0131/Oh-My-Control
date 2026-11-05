#pragma once
#define ATOMIC_BLOCK(ignore) for (bool nativeAtomic=true; nativeAtomic; nativeAtomic=false)
