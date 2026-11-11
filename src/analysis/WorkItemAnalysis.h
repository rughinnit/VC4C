/*
 * Author: doe300
 *
 * See the file "LICENSE" for the full license governing this code.
 */
#ifndef VC4C_WORK_ITEM_ANALYSIS
#define VC4C_WORK_ITEM_ANALYSIS

#include "../Optional.h"
#include "../performance.h"
#include "config.h"

#include <string>

namespace vc4c
{
    class BasicBlock;
    class Method;
    class Module;

    namespace intermediate
    {
        class MemoryInstruction;
    } /* namespace intermediate */

    namespace analysis
    {
        /**
         * Checks whether the given kernel-function can be executed with multiple work-items merged into a single QPU
         * execution (one work-item per SIMD lane along dimension 0).
         *
         * Returns an empty optional if merging is supported, otherwise the reason why it is not.
         *
         * NOTE: This check must be run on the intermediate code after the called functions have been inlined and
         * before the other normalization steps, since it relies on the calls to the work-item built-ins (which are
         * intrinsified by the normalization) with their actual (constant or dynamic) dimension arguments.
         */
        Optional<std::string> checkWorkItemMerging(const Module& module, const Method& kernel, uint8_t factor);

        /**
         * Finds all memory reads of the given method which access an address that is identical for all merged
         * work-items.
         *
         * Such a read loads exactly one element which is the value of every SIMD lane, so the memory lowering can
         * perform a single lookup and broadcast the loaded value into all lanes, instead of a gather (for a per-lane
         * address) or a contiguous multi-element load (which would read the following elements).
         *
         * Only the read addresses which are provably lane-uniform are returned, i.e. reads whose address is
         * lane-dependent in an unknown way (e.g. a phi node without a uniform step) are not contained.
         *
         * NOTE: Like #checkWorkItemMerging(), this must be run on the intermediate code after inlining, since it
         * follows the address calculation which is (partially) contained in the inlined functions.
         */
        FastSet<const intermediate::MemoryInstruction*> findLaneUniformMemoryReads(const Method& method);

        class ControlFlowGraph;

        enum class WorkItemCondition
        {
            LOCAL_ID_X,
            LOCAL_ID_Y,
            LOCAL_ID_Z,
            LOCAL_ID_SCALAR,
            LOCAL_SIZE_X,
            LOCAL_SIZE_Y,
            LOCAL_SIZE_Z,
            LOCAL_SIZE_SCALAR,
            GLOBAL_ID_X,
            GLOBAL_ID_Y,
            GLOBAL_ID_Z,
            GROUP_ID_X,
            GROUP_ID_Y,
            GROUP_ID_Z
        };

        /**
         * Storage container for information about the active work-items of the associated basic block.
         *
         * NOTE: Depending on the jump conditions into the associated basic block, some entries might be explicitly
         * enabled or disabled. Thus, both the active and inactive elements member need to be checked!
         */
        struct ActiveWorkItems
        {
            WorkItemCondition condition;

            /**
             * The active elements, interpreted according to the work-item condition, i.e.:
             *
             * - For LOCAL_ID_XYZ, this is the list of active local work-items (within a local group)
             * - For GLOBAL_ID_XYZ, this is the list of active global work-items (across all work-groups)
             * - For GROUP_ID_XYZ, this is the list of active work-groups (all work-items of the selected groups)
             */
            SortedSet<std::size_t> activeElements;

            /**
             * The inactive elements, interpreted according to the work-item condition, i.e.:
             *
             * - For LOCAL_ID_XYZ, this is the list of inactive local work-items (within a local group)
             * - For GLOBAL_ID_XYZ, this is the list of inactive global work-items (across all work-groups)
             * - For GROUP_ID_XYZ, this is the list of inactive work-groups (no work-items of the selected groups)
             */
            SortedSet<std::size_t> inactiveElements;

            inline bool isActive(std::size_t element) const noexcept
            {
                return activeElements.find(element) != activeElements.end() ||
                    inactiveElements.find(element) == inactiveElements.end();
            }

            inline bool isSingleElement() const noexcept
            {
                return activeElements.size() == 1 && inactiveElements.empty();
            }

            bool isWorkItemCondition() const noexcept;

            /**
             * Tries to merge the two active work item constraints.
             *
             * If they constraints cannot be merged (e.g. they have different conditions or contain opposite
             * active/inactive elements), an empty value is returned.
             */
            Optional<ActiveWorkItems> mergeWith(const ActiveWorkItems& other) const;

            std::string to_string() const;
        };

        /**
         * Analyzed the given control flow graph and tries to determine for each basic block:
         *
         * 1. whether it is only accessed by specific work-items
         * 2. the condition and included/excluded work-items that can execute this block
         *
         * NOTE: The result only lists the basic block where work-item-dependent condition could be determined. Thus,
         * any basic block not listed in the resulting map should be considered to be (possibly) accessed by all
         * work-items.
         */
        FastMap<const BasicBlock*, ActiveWorkItems> determineActiveWorkItems(
            const Method& method, ControlFlowGraph& cfg);
    } // namespace analysis
} // namespace vc4c

#endif /* VC4C_WORK_ITEM_ANALYSIS */
