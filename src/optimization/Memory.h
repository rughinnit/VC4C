/*
 * Author: doe300
 *
 * See the file "LICENSE" for the full license governing this code.
 */

#ifndef VC4C_OPTIMIZATION_MEMORY_H
#define VC4C_OPTIMIZATION_MEMORY_H

#include <cstdint>

namespace vc4c
{
    class Method;
    class Module;
    struct Configuration;

    namespace optimizations
    {
        /**
         * Maps all intermediate memory access instructions to the actual hardware instructions used to perform the
         * given memory access.
         *
         * NOTE: This is not actually an optimization (but rather a normalization) step, but for allowing some
         * optimizations to access the more detailed information provided by MemoryAccessInstructions and other
         * optimizations to rewrite the low-level access functions requires this to be run in between the optimization
         * steps.
         */
        std::size_t lowerMemoryAccess(const Module& module, Method& method, const Configuration& config);

        /**
         * Combine consecutive configuration of VPW/VPR with the same settings
         *
         * In detail, this combines VPM read/writes of uniform type of access (read or write), uniform data-type and
         * consecutive memory-addresses
         *
         * NOTE: Combining VPM accesses merges their mutex-lock blocks which can cause other QPUs to stall for a long
         * time. Also, this optimization currently only supports access memory <-> QPU, data exchange between only
         * memory and VPM are not optimized
         */
        std::size_t groupVPMAccess(const Module& module, Method& method, const Configuration& config);

        /**
         * Tries to find and group memory accesses to reduce the number of accesses while increasing utilization.
         */
        std::size_t groupTMUAccess(const Module& module, Method& method, const Configuration& config);

        /**
         * Tries to find TMU loads within loops where we can pre-calculate the address for loads in the next loop
         * iteration and thus we can pre-fetch the data loaded for the next loop iteration into the TMU FIFO.
         */
        std::size_t prefetchTMULoads(const Module& module, Method& method, const Configuration& config);

        /**
         * Re-orders the TMU requests and value reads inside single-block loop bodies, so the memory lookups of a loop
         * iteration are started as early as possible and their results are consumed as late as possible, hiding the
         * lookup latency behind the calculations of the loop body.
         *
         * At most 2 requests per TMU are kept in flight (the measured hardware optimum, both TMUs are used in
         * parallel), the in-order FIFOs of the TMUs are respected by keeping the issue order and the pop order of each
         * TMU as well as keeping each pop directly in front of the read of r4 it belongs to.
         *
         * NOTE: This optimization re-orders memory accesses and is therefore disabled by default. It can be enabled
         * via the environment variable VC4C_PIPELINE_LOADS=<number of requests in flight per TMU (1-4), default 2> or
         * the optimization pass name "pipeline-tmu-loads" (e.g. on the command line).
         */
        std::size_t pipelineTMULoads(const Module& module, Method& method, const Configuration& config);

        /**
         * Tries to find and group memory accesses to reduce the number of accesses while increasing utilization.
         */
        std::size_t groupLoweredRegisterAccess(const Module& module, Method& method, const Configuration& config);
    } // namespace optimizations
} // namespace vc4c

#endif /* VC4C_OPTIMIZATION_MEMORY_H */
