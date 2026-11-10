/*
 * Author: doe300
 *
 * See the file "LICENSE" for the full license governing this code.
 */

#ifndef VC4C_COMPILER_INSTANCE_H
#define VC4C_COMPILER_INSTANCE_H

#include "Compiler.h"
#include "Module.h"

#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace vc4c
{
    namespace qpu_asm
    {
        struct RegisterFixupStep;
    } // namespace qpu_asm

    /**
     * Wrapper container around the single compilation steps.
     *
     * The main purpose of this container is to allow extensions/modifications of the compilation flow for tests.
     */
    struct CompilerInstance
    {
        Configuration moduleConfig;
        Module module;

        explicit CompilerInstance(const Configuration& config);

        void precompileAndParseInput(const CompilationData& input, const std::string& options = "");
        void parseInput(const CompilationData& input);
        /**
         * Enables the merging of independent work-items (one work-item per SIMD lane) for all kernel-functions which
         * support it.
         *
         * This is called by the normalizer after inlining the called functions and before the other normalization
         * steps, since it changes how the work-item built-ins are intrinsified and the check requires the fully
         * inlined kernel.
         */
        void applyWorkItemMerging();
        void normalize(bool dropNonKernels = true);
        void normalize(const std::set<std::string>& selectedSteps, bool dropNonKernels = true);
        void optimize();
        void optimize(const std::vector<std::string>& selectedPasses);
        void adjust(const std::set<std::string>& selectedSteps = {});
        std::size_t generateCode(std::ostream& output);
        std::size_t generateCode(std::ostream& output, const std::vector<qpu_asm::RegisterFixupStep>& customSteps);
        std::pair<CompilationData, std::size_t> generateCode(const Optional<std::string>& outputFile = {});
    };
} // namespace vc4c

#endif /* VC4C_COMPILER_INSTANCE_H */
