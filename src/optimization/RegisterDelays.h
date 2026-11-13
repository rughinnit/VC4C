/*
 * See the file "LICENSE" for the full license governing this code.
 */

#ifndef VC4C_REGISTER_DELAYS_H
#define VC4C_REGISTER_DELAYS_H

#include "../performance.h"

namespace vc4c
{
    class Local;
    class Method;
    class Module;
    struct Configuration;
    struct Register;

    namespace qpu_asm
    {
        struct DecoratedInstruction;
    } // namespace qpu_asm

    namespace optimizations
    {
        /**
         * Inserts the delay instructions required by the QPU timing rules for every write-read pair in the final
         * instruction stream.
         *
         * The delays inserted by the pass #splitReadAfterWrites() are decided before the register allocation, based on
         * the heuristic whether a local will be mapped to an accumulator (see BasicBlock#isLocallyLimited()). The
         * register allocation can invalidate that assumption: locals then end up in the general-purpose register files,
         * for which the QPU does not provide any forwarding, and reading them in the instruction following the write
         * returns the previous value. In addition, filling a delay NOP by moving another instruction into its place can
         * create new write-read pairs.
         *
         * This pass is the backstop for exactly that hole. It runs after the register-mapping and after all
         * peephole-optimizations and checks every write-read pair of the final instruction stream against the delay
         * model documented in RegisterDelays.cpp, inserting a NOP where the model requires a larger distance.
         *
         * NOTE: This pass operates on the (register-mapped) intermediate instructions and therefore MUST run before the
         * instructions are converted to machine code, since inserting instructions invalidates the label mapping.
         */
        void insertMissingRegisterDelays(const Module& module, Method& kernel, const Configuration& config,
            const FastMap<const Local*, Register>& registerMap);

        /**
         * Verifies every write-read pair of the final machine-code instructions against the delay model documented in
         * RegisterDelays.cpp.
         *
         * This is the last check before the machine-code is written out. Any violation is a miscompile, since the QPU
         * will silently compute with the previous value, so a violation aborts the compilation with a diagnostic
         * instead of emitting wrong code.
         */
        void verifyRegisterDelays(const Module& module, Method& kernel,
            const FastAccessList<qpu_asm::DecoratedInstruction>& instructions);
    } // namespace optimizations
} // namespace vc4c

#endif /* VC4C_REGISTER_DELAYS_H */
