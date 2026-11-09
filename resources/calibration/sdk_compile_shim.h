// macOS libc++ pulls in the M_PI macro through standard headers. Preload them
// and undefine that macro so the original SDK's same-named member can compile.
// This changes no protocol formula or SDK source.
#include <cmath>
#include <cstdint>
#include <vector>
#include <memory>
#include <string>
#undef M_PI
