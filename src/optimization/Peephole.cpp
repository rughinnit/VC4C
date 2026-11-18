
#include "Peephole.h"

#include "../InstructionWalker.h"
#include "../Locals.h"
#include "../Method.h"
#include "../Profiler.h"
#include "../Values.h"
#include "../intermediate/Helper.h"
#include "../intermediate/IntermediateInstruction.h"
#include "Combiner.h"
#include "log.h"

#include <cstdlib>

using namespace vc4c;
using namespace vc4c::optimizations;

using RegisterMap = FastMap<const Local*, Register>;

static InstructionWalker lookAhead(InstructionWalker it)
{
    it.nextInMethod();
    while(!it.isEndOfMethod())
    {
        if(it.has() && it->mapsToASMInstruction())
            break;
        it.nextInMethod();
    }
    return it;
}

static InstructionWalker lookAheadInBlock(InstructionWalker it)
{
    it.nextInBlock();
    while(!it.isEndOfBlock())
    {
        if(it.has() && it->mapsToASMInstruction())
            break;
        it.nextInBlock();
    }
    return it;
}

static InstructionWalker lookBackInBlock(InstructionWalker it)
{
    it.previousInBlock();
    while(!it.isEndOfBlock() && !it.isStartOfBlock())
    {
        if(it.has() && it->mapsToASMInstruction())
            break;
        it.previousInBlock();
    }
    return it;
}

static InstructionWalker findSingleReaderInBlock(InstructionWalker it, InstructionWalker endIt, const Local* loc,
    const RegisterMap& registerMap, const Register& conflictingRegister)
{
    auto locIt = registerMap.find(loc);
    if(locIt == registerMap.end())
        // not found, abort
        return endIt;

    while(!(it = lookAheadInBlock(it)).isEndOfBlock())
    {
        if(it->writesLocal(loc))
            return endIt;
        if(it->readsLocal(loc))
            return it;
        bool readsRegister = false;
        bool isAborted = false;
        it.forAllInstructions([&](const intermediate::IntermediateInstruction& inst) {
            for(const auto& arg : inst.getArguments())
            {
                if(auto reg = arg.checkRegister())
                    readsRegister = readsRegister || *reg == locIt->second;
                else if(auto loc = arg.checkLocal())
                {
                    auto argLocIt = registerMap.find(loc);
                    if(argLocIt == registerMap.end())
                        // local not mapped to register -> unknown state -> abort
                        isAborted = true;
                    else
                        readsRegister = readsRegister || argLocIt->second == locIt->second;
                }
            }
            if(inst.writesRegister(conflictingRegister))
                // overwrites input register -> abort
                isAborted = true;
            if(auto outLoc = inst.checkOutputLocal())
            {
                auto outIt = registerMap.find(outLoc);
                if(outIt == registerMap.end() || outIt->second == conflictingRegister)
                    // overwrites input register -> abort
                    isAborted = true;
            }
        });

        if(isAborted)
            break;
        if(readsRegister)
            return it;
    }
    return endIt;
}

static bool canRemoveInstructionBetween(
    InstructionWalker startIt, InstructionWalker endIt, const RegisterMap& registerMap)
{
    if(!startIt.has() || !endIt.has() || endIt.isEndOfMethod())
        return false;

    bool isAborted = false;
    SortedSet<Register> writtenRegisters;
    startIt.forAllInstructions([&](const intermediate::IntermediateInstruction& inst) {
        if(auto reg = inst.checkOutputRegister())
            writtenRegisters.emplace(*reg);
        else if(auto loc = inst.checkOutputLocal())
        {
            auto locIt = registerMap.find(loc);
            if(locIt == registerMap.end())
                isAborted = true;
            else
                writtenRegisters.emplace(locIt->second);
        }
    });

    if(isAborted)
        return false;

    std::size_t numInstructionsInBetween = 0;
    auto getRegister = [&](const Value& val) -> Optional<Register> {
        if(auto reg = val.checkRegister())
            return *reg;
        if(auto loc = val.checkLocal())
        {
            auto regIt = registerMap.find(loc);
            if(regIt != registerMap.end())
                return regIt->second;
        }
        return {};
    };
    for(auto checkIt = startIt.copy().nextInBlock(); !checkIt.isEndOfBlock() && checkIt != endIt; checkIt.nextInBlock())
    {
        // make sure we do not access any of the written registers in between, neither read from it (in which case we
        // need it defined) nor write to it (in which case we would override any previous value)
        if(checkIt.has() && checkIt->mapsToASMInstruction())
        {
            ++numInstructionsInBetween;
            std::vector<const intermediate::IntermediateInstruction*> instructions{checkIt.get()};
            if(auto combined = checkIt.get<const intermediate::CombinedOperation>())
                instructions = {combined->getFirstOp(), combined->getSecondOp()};
            for(auto inst : instructions)
            {
                for(const auto& arg : inst->getArguments())
                {
                    if(auto reg = getRegister(arg))
                    {
                        if(writtenRegisters.find(*reg) != writtenRegisters.end())
                            return false;
                    }
                }
                if(auto out = inst->getOutput())
                {
                    if(auto reg = getRegister(*out))
                    {
                        if(writtenRegisters.find(*reg) != writtenRegisters.end())
                            return false;
                    }
                }
            }
        }
    }

    SortedSet<Register> readRegisters;
    endIt.forAllInstructions([&](const intermediate::IntermediateInstruction& inst) {
        for(const auto& arg : inst.getArguments())
        {
            if(auto reg = arg.checkRegister())
                readRegisters.emplace(*reg);
            else if(auto loc = arg.checkLocal())
            {
                auto locIt = registerMap.find(loc);
                if(locIt == registerMap.end())
                    isAborted = true;
                else
                    readRegisters.emplace(locIt->second);
            }
        }
    });

    if(isAborted)
        return false;

    for(const auto& reg : writtenRegisters)
    {
        bool isRead = readRegisters.find(reg) != readRegisters.end();
        if(!isRead && (reg == REG_REPLICATE_ALL || reg == REG_REPLICATE_QUAD))
            isRead = readRegisters.find(REG_ACC5) != readRegisters.end();
        if(isRead && reg.isGeneralPurpose())
            return false;
        if(isRead && (startIt->hasPackMode() || endIt->hasUnpackMode()))
            return false;
        if(isRead && numInstructionsInBetween < 2 && endIt->getVectorRotation())
            return false;
    }

    return numInstructionsInBetween > 0 || !intermediate::needsDelay(startIt.get(), endIt.get());
}

static Optional<Register> getSingleRegister(
    const intermediate::IntermediateInstruction& inst, const RegisterMap& registerMap)
{
    SortedSet<Register> usedRegisters;
    inst.forUsedLocals([&usedRegisters, &registerMap](
                           const Local* loc, LocalUse::Type type, const intermediate::IntermediateInstruction&) {
        auto locIt = registerMap.find(loc);
        if(locIt != registerMap.end())
            usedRegisters.emplace(locIt->second);
        else
            // to be on the safe side, treat "not-mapped" locals as mapping to NOP which disables removal of
            // instruction for now
            usedRegisters.emplace(REG_NOP);
    });
    return usedRegisters.size() == 1 ? *usedRegisters.begin() : Optional<Register>{};
}

static const intermediate::Operation* getSimpleLocalMove(const intermediate::Operation* op)
{
    if(op && op->isSimpleMove() && !op->hasConditionalExecution() && op->readsLocal() && op->checkOutputLocal())
        return op;
    return nullptr;
}

void optimizations::removeObsoleteInstructions(
    const Module& module, Method& kernel, const Configuration& config, const RegisterMap& registerMap)
{
    logging::logLazy(logging::Level::DEBUG, [&]() {
        logging::debug() << logging::endl;
        logging::debug() << "Running peephole pass: RemoveInstructions" << logging::endl;
    });

    PROFILE_COUNTER(
        vc4c::profiler::COUNTER_OPTIMIZATION, "PeepholeRemoveInstructions (before)", kernel.countInstructions());
    PROFILE_START(PeepholeRemoveInstructions);
    std::size_t numChanges = 0;

    auto it = kernel.walkAllInstructions();
    auto lastInstruction = it;
    while(!it.isEndOfMethod())
    {
        if(it.isEndOfBlock() || !it.has() || !it->mapsToASMInstruction())
        {
            it.nextInMethod();
            continue;
        }
        auto nop = it.get<intermediate::Nop>();
        auto combined = it.get<intermediate::CombinedOperation>();
        if(it->hasDecoration(intermediate::InstructionDecorations::MANDATORY_DELAY) &&
            (!nop || nop->type != intermediate::DelayType::WAIT_REGISTER) && !combined)
        {
            // The instruction is mapped, but we just don't touch it. We need to skip this for NOPs otherwise, we cannot
            // remove any of them. Also skip this for combined operations, since we do not completely remove them and
            // therefore do not violate the delay.
            lastInstruction = it;
            it.nextInMethod();
            continue;
        }
        auto nextIt = lookAhead(it);
        if(nop && nop->type == intermediate::DelayType::WAIT_REGISTER && !nop->hasSideEffects() &&
            !nextIt.isEndOfMethod() && nextIt.has() &&
            canRemoveInstructionBetween(lastInstruction, nextIt, registerMap))
        {
            /*
             * This NOP was inserted to split-read-after writes (mostly for longer-living locals, so they can be mapped
             * to physical registers), but the locals are mapped to accumulators, so there is no need for the NOP
             * anymore.
             */
            CPPLOG_LAZY(logging::Level::DEBUG,
                log << "Removing NOP inserted for register-delay which is not needed between '"
                    << lastInstruction->to_string() << "' and '" << nextIt->to_string() << "': " << it->to_string()
                    << logging::endl);
            it.erase();
            ++numChanges;
            continue;
        }
        if(it->isSimpleMove() && it->checkOutputLocal() && it->getMoveSource()->checkLocal())
        {
            auto reg = getSingleRegister(*it.get(), registerMap);
            if(reg && canRemoveInstructionBetween(lastInstruction, nextIt, registerMap))
            {
                // Input and output are mapped to the same register and there is no need for a delay instruction between
                // the previous and next instruction -> remove the move completely
                CPPLOG_LAZY(logging::Level::DEBUG,
                    log << "Removing simple move with input and output mapped to the same register: " << it->to_string()
                        << " (register " << reg->to_string() << ')' << logging::endl);
                it.erase();
                ++numChanges;
                continue;
            }
        }
        if(it->isSimpleMove() && !it->hasConditionalExecution() && it->checkOutputLocal() &&
            it->getMoveSource()->checkLocal())
        {
            auto moveOut = it->checkOutputLocal();
            auto moveIn = it->getMoveSource()->checkLocal();
            auto inIt = registerMap.find(moveIn);
            auto outIt = registerMap.find(moveOut);
            if(moveOut && moveIn && moveOut->getSingleWriter() == it.get() &&
                moveOut->countUsers(LocalUse::Type::READER) == 1 && inIt != registerMap.end() &&
                outIt != registerMap.end() && inIt->second.file == outIt->second.file &&
                (outIt->second != REG_ACC5 ||
                    it->hasDecoration(intermediate::InstructionDecorations::IDENTICAL_ELEMENTS)))
            {
                /*
                 * A simple move from local to local where the output is written and read exactly once. Furthermore, the
                 * in/out locals are mapped to registers in the same register file -> where one can be used, the other
                 * one can be used instead -> we can remove the move and directly use its input. Moves to the
                 * replication register are only removed if the input is already a splat value.
                 */
                if(!nextIt.isEndOfMethod() && it.getBasicBlock() == nextIt.getBasicBlock() &&
                    nextIt->readsLocal(moveOut) && canRemoveInstructionBetween(lastInstruction, nextIt, registerMap))
                {
                    /*
                     * Next instruction is the single reader of the move output and there is no need for a delay
                     * between the previous and the next instruction -> we can remove the move and directly use its
                     * input in the next instruction's input. For now only check for both instructions in the same
                     * block, since otherwise we would need to check for register re-usage in case of loops.
                     */
                    CPPLOG_LAZY(logging::Level::DEBUG,
                        log << "Removing simple move with input and output mapped to the same register file and single "
                               "read in next instruction: "
                            << it->to_string() << " (registers " << inIt->second.to_string() << " and "
                            << outIt->second.to_string() << ')' << logging::endl);
                    it.erase();
                    nextIt->replaceLocal(moveOut, moveIn, LocalUse::Type::READER);
                    ++numChanges;
                    continue;
                }
                auto readerIt =
                    findSingleReaderInBlock(it, it.getBasicBlock()->walkEnd(), moveOut, registerMap, inIt->second);
                auto beforeReaderIt =
                    !readerIt.isStartOfBlock() ? readerIt.copy().previousInBlock() : Optional<InstructionWalker>{};
                if(!readerIt.isEndOfBlock() && readerIt->readsLocal(moveOut) && !readerIt->hasUnpackMode() &&
                    !readerIt->getVectorRotation() && !nextIt.isEndOfMethod() &&
                    canRemoveInstructionBetween(lastInstruction, readerIt, registerMap) && beforeReaderIt &&
                    canRemoveInstructionBetween(*beforeReaderIt, readerIt, registerMap) &&
                    !(*beforeReaderIt)->writesLocal(moveIn))
                {
                    CPPLOG_LAZY(logging::Level::DEBUG,
                        log << "Removing simple move with input and output mapped to the same register file and single "
                               "read in a successive instruction: "
                            << it->to_string() << " (registers " << inIt->second.to_string() << " and "
                            << outIt->second.to_string() << ')' << logging::endl);
                    it.erase();
                    readerIt->replaceLocal(moveOut, moveIn, LocalUse::Type::READER);
                    ++numChanges;
                    continue;
                }
            }
        }
        if(combined)
        {
            const intermediate::Operation* simpleMoveOp = nullptr;
            if(auto first = getSimpleLocalMove(combined->getFirstOp()))
                simpleMoveOp = first;
            else if(auto second = getSimpleLocalMove(combined->getSecondOp()))
                simpleMoveOp = second;
            if(auto reg = simpleMoveOp ? getSingleRegister(*simpleMoveOp, registerMap) : Optional<Register>{})
            {
                /*
                 * The combined operations contains a simple move where the input and output register is the same -> we
                 * can remove this move.
                 * This does not directly benefit us performance-wise, but allows for more instructions to be combined
                 * in a successive peephole-combination optimization step.
                 */
                CPPLOG_LAZY(logging::Level::DEBUG,
                    log << "Removing simple move with input and output mapped to the same register '"
                        << simpleMoveOp->to_string() << "' (register " << reg->to_string()
                        << ") as part of a combined instruction: " << combined->to_string() << logging::endl);
                if(combined->getFirstOp() == simpleMoveOp)
                {
                    auto tmp = std::move(combined->splitUp().second);
                    tmp->copyExtrasFrom(*combined);
                    it.reset(std::move(tmp));
                    ++numChanges;
                }
                else if(combined->getSecondOp() == simpleMoveOp)
                {
                    auto tmp = std::move(combined->splitUp().first);
                    tmp->copyExtrasFrom(*combined);
                    it.reset(std::move(tmp));
                    ++numChanges;
                }
            }
        }
        lastInstruction = it;
        it.nextInMethod();
    }

    PROFILE_END(PeepholeRemoveInstructions);
    PROFILE_COUNTER_WITH_PREV(
        vc4c::profiler::COUNTER_OPTIMIZATION, "PeepholeRemoveInstructions (after)", kernel.countInstructions());
    {
        PROFILE_COUNTER(vc4c::profiler::COUNTER_OPTIMIZATION, "PeepholeRemoveInstructions (changes)", numChanges);
    }
}

void optimizations::combineRegisterMappedOperations(const Module& module, Method& kernel, const Configuration& config,
    const FastMap<const Local*, Register>& registerMap)
{
    logging::logLazy(logging::Level::DEBUG, [&]() {
        logging::debug() << logging::endl;
        logging::debug() << "Running peephole pass: CombineInstructions" << logging::endl;
    });

    PROFILE_COUNTER(
        vc4c::profiler::COUNTER_OPTIMIZATION, "PeepholeCombineInstructions (before)", kernel.countInstructions());
    PROFILE_START(PeepholeCombineInstructions);
    std::size_t numChanges = 0;

    for(auto& block : kernel)
    {
        auto it = block.walk();
        auto nextIt = lookAheadInBlock(it);
        while(!it.isEndOfBlock() && !nextIt.isEndOfBlock())
        {
            if(!it.has() || !it->mapsToASMInstruction() || !nextIt.has() || !nextIt->mapsToASMInstruction())
            {
                it.nextInBlock();
                nextIt = lookAheadInBlock(it);
                continue;
            }

            bool conditionsMet = true;
            auto move = it.get<intermediate::MoveOperation>();
            auto op = it.get<intermediate::Operation>();
            auto nextMove = nextIt.get<intermediate::MoveOperation>();
            auto nextOp = nextIt.get<intermediate::Operation>();
            if((!move && !op) || (!nextMove && !nextOp))
                conditionsMet = false;

            if(conditionsMet)
            {
                MergeConditionData data{&registerMap};
                conditionsMet = std::all_of(MERGE_CONDITIONS.begin(), MERGE_CONDITIONS.end(),
                    [op, nextOp, move, nextMove, &data](
                        const MergeCondition& cond) -> bool { return cond(op, nextOp, move, nextMove, data); });
            }
            if(conditionsMet)
            {
                auto checkIt = lookAheadInBlock(nextIt);
                if(checkIt.isEndOfBlock() || !checkIt.has() || !canRemoveInstructionBetween(it, checkIt, registerMap))
                    conditionsMet = false;
            }
            if(conditionsMet)
            {
                auto checkIt = lookBackInBlock(it);
                if(checkIt.isEndOfBlock() || !checkIt.has() ||
                    !canRemoveInstructionBetween(checkIt, nextIt, registerMap))
                    conditionsMet = false;
            }

            if(conditionsMet && combineOperationsInner(it, nextIt))
                ++numChanges;

            it.nextInBlock();
            nextIt = lookAheadInBlock(it);
        }
    }
    PROFILE_END(PeepholeCombineInstructions);
    PROFILE_COUNTER_WITH_PREV(
        vc4c::profiler::COUNTER_OPTIMIZATION, "PeepholeCombineInstructions (after)", kernel.countInstructions());
    {
        PROFILE_COUNTER(vc4c::profiler::COUNTER_OPTIMIZATION, "PeepholeCombineInstructions (changes)", numChanges);
    }
}

/*
 * =====================================================================================================================
 * Windowed packing of ALU operations after the register-mapping
 * =====================================================================================================================
 *
 * Each QPU instruction word executes one operation on the add ALU and one on the mul ALU. #combineRegisterMappedOperations()
 * already combines such operation pairs, but it only looks at direct neighbours. The instruction order produced by the
 * register-allocation is not necessarily the one which allows both ALUs to be used together: e.g. a delay instruction
 * (or a copy required by a vector rotation) can sit between two otherwise independent operations, and the pair stays
 * uncombined even though the second operation could be executed earlier.
 *
 * This pass therefore looks ahead up to VC4C_PACK_WINDOW (default: 6) instructions within the same basic block for an
 * operation which can be executed on the other ALU, verifies that it can be moved up in front of the instructions it
 * passes, moves it to be a direct successor of the first operation and then combines the two with the same conditions
 * (see MERGE_CONDITIONS) and the same code (#combineOperationsInner()) as the direct-neighbour case.
 *
 * A move is only considered safe if
 * - both operations are plain ALU operations: no flags, no pack/unpack mode, no vector rotation, no signal, no side
 *   effects and no special-purpose register. Memory (TMU/VPM) and periphery accesses are never moved, so a memory
 *   request can never be moved in front of the instruction filling its address register,
 * - no instruction which is passed accesses any of the registers read or written by the moved operation,
 * - no label or branch is crossed (the window is inside a single basic block and stops at any other kind of
 *   instruction), and
 * - the combination does not create a write-read pair which violates the register delay model (see
 *   optimizations#insertMissingRegisterDelays()). Without that last check the combination would only move an operation
 *   into the delay slot of the other one and the delay instruction would be re-inserted afterwards, not saving anything.
 *
 * Since the register-mapping is fixed at this point, the operations are only moved and their instruction words shared,
 * no register assignment is changed.
 */
namespace
{
    /*
     * The physical registers read and written by a single operation.
     */
    struct PackAccess
    {
        SortedSet<Register> reads;
        SortedSet<Register> writes;
    };

    /*
     * Maps a register to the physical register it addresses, so that different representations of the same register
     * (e.g. the replication registers and accumulator r5) are recognized as the same value.
     *
     * Returns nothing for registers which are not plain general-purpose or accumulator registers, since those are shared
     * resources and must not be moved around.
     */
    static Optional<Register> toPhysicalRegister(const Register& reg)
    {
        if(reg.num >= 32)
        {
            if(reg.num == REG_ACC5.num || reg.num == REG_REPLICATE_ALL.num || reg.num == REG_REPLICATE_QUAD.num)
                // all replication registers are the physical register r5
                return REG_ACC5;
            if(reg.file == RegisterFile::ACCUMULATOR && reg.num >= REG_ACC0.num && reg.num <= REG_ACC3.num)
                // the accumulators r0-r3 are the registers 0-3 of register file A
                return Register{RegisterFile::PHYSICAL_A, static_cast<unsigned char>(reg.num - REG_ACC0.num)};
            // any other special-purpose register (TMU/VPM/uniforms/...) is a shared resource
            return {};
        }
        return reg;
    }

    static Optional<PackAccess> getPackAccess(
        const intermediate::IntermediateInstruction& inst, const RegisterMap& registerMap)
    {
        PackAccess access;
        auto addRegister = [&registerMap](const Value& val, SortedSet<Register>& regs) -> bool {
            Optional<Register> reg;
            if(auto tmp = val.checkRegister())
                reg = *tmp;
            else if(auto loc = val.checkLocal())
            {
                auto it = registerMap.find(loc);
                if(it == registerMap.end())
                    // not mapped to a register -> do not touch this instruction
                    return false;
                reg = it->second;
            }
            else
                // literal/immediate value, does not read a register
                return true;
            auto physical = toPhysicalRegister(*reg);
            if(!physical)
                return false;
            if(physical->num != REG_NOP.num)
                regs.emplace(*physical);
            return true;
        };

        for(const auto& arg : inst.getArguments())
        {
            if(!addRegister(arg, access.reads))
                return {};
        }
        if(auto out = inst.getOutput())
        {
            if(!addRegister(*out, access.writes))
                return {};
        }
        return access;
    }

    /*
     * Whether the instruction is a plain ALU operation which can be moved within its basic block and combined with
     * another operation without any side effects.
     */
    static bool isMovableALUOp(const intermediate::IntermediateInstruction& inst, const RegisterMap& registerMap)
    {
        if(!inst.mapsToASMInstruction())
            return false;
        if(!dynamic_cast<const intermediate::Operation*>(&inst) &&
            !dynamic_cast<const intermediate::MoveOperation*>(&inst))
            // no NOP, branch, load/store, semaphore or already combined operation
            return false;
        if(dynamic_cast<const intermediate::VectorRotation*>(&inst))
            // vector rotations have their own delay rules and cannot be combined (see VectorRotation#combineWith())
            return false;
        if(inst.hasSideEffects() || inst.hasConditionalExecution() || inst.doesSetFlag())
            // depends on or sets the flags, which cannot be moved across another flag access
            return false;
        if(inst.hasPackMode() || inst.hasUnpackMode() || inst.getVectorRotation())
            // pack/unpack modes and rotations have additional delay rules
            return false;
        if(inst.getSignal() != SIGNAL_NONE)
            // e.g. a signal for the periphery
            return false;
        for(const auto deco : {intermediate::InstructionDecorations::MANDATORY_DELAY,
                intermediate::InstructionDecorations::PHI_NODE,
                intermediate::InstructionDecorations::ELEMENT_INSERTION,
                intermediate::InstructionDecorations::PER_LANE_ADDRESS})
        {
            if(inst.hasDecoration(deco))
                return false;
        }
        // only locals and immediate values as arguments, no (shared) register values
        for(const auto& arg : inst.getArguments())
        {
            if(!arg.checkLocal() && !arg.checkImmediate())
                return false;
        }
        if(auto out = inst.getOutput())
        {
            if(!out->checkLocal())
                return false;
        }
        return getPackAccess(inst, registerMap).has_value();
    }

    /*
     * Whether the two operations access any register in common, in which case their order is not arbitrary.
     */
    static bool accessesOverlap(const PackAccess& first, const PackAccess& second)
    {
        for(const auto& reg : first.writes)
        {
            if(second.writes.find(reg) != second.writes.end() || second.reads.find(reg) != second.reads.end())
                return true;
        }
        for(const auto& reg : first.reads)
        {
            if(second.writes.find(reg) != second.writes.end())
                return true;
        }
        return false;
    }

    /*
     * Whether the two operations can be executed in the same instruction word at all. This is checked before the
     * operation is moved, so that the instruction order is not changed for a pair which cannot be combined anyway.
     */
    static bool isCombinablePair(const intermediate::IntermediateInstruction& first,
        const intermediate::IntermediateInstruction& second, const RegisterMap& registerMap)
    {
        // both operations must be executable on different ALUs (moves and operations which run on both ALUs can be
        // mapped to the ALU which is still free, see #combineOperationsInner())
        auto firstOp = dynamic_cast<const intermediate::Operation*>(&first);
        auto secondOp = dynamic_cast<const intermediate::Operation*>(&second);
        const bool firstRunsOnAnyALU =
            firstOp == nullptr || (firstOp->op.runsOnAddALU() && firstOp->op.runsOnMulALU());
        const bool secondRunsOnAnyALU =
            secondOp == nullptr || (secondOp->op.runsOnAddALU() && secondOp->op.runsOnMulALU());
        if(!firstRunsOnAnyALU && !secondRunsOnAnyALU &&
            firstOp->op.runsOnAddALU() == secondOp->op.runsOnAddALU())
            return false;

        auto firstAccess = getPackAccess(first, registerMap);
        auto secondAccess = getPackAccess(second, registerMap);
        if(!firstAccess || !secondAccess || accessesOverlap(*firstAccess, *secondAccess))
            return false;
        // the combined instruction can only write one register per register file (accumulators and other special
        // registers can be written by both ALUs)
        for(const auto& firstOut : firstAccess->writes)
        {
            for(const auto& secondOut : secondAccess->writes)
            {
                if(firstOut.file == secondOut.file && firstOut.isGeneralPurpose() && secondOut.isGeneralPurpose())
                    return false;
            }
        }
        return true;
    }

    /*
     * The minimum number of instruction words between writing and reading the given register, as modeled by
     * optimizations#insertMissingRegisterDelays() (see RegisterDelays.cpp for the full rules and the citations).
     */
    static std::size_t minimumReadDistance(const Register& reg, bool writtenWithPackMode, bool readByRotationOrUnpack)
    {
        if(writtenWithPackMode || readByRotationOrUnpack)
            return 2;
        if(reg.file == RegisterFile::ACCUMULATOR && reg.num >= REG_ACC0.num && reg.num <= REG_ACC3.num)
            // the accumulators can be read by the very next instruction
            return 1;
        // there are no forwarding paths for the general-purpose register files
        return 2;
    }

    static bool readsWithRotationOrUnpack(const intermediate::IntermediateInstruction& inst)
    {
        return inst.getVectorRotation().has_value() || inst.hasUnpackMode();
    }

    /*!
     * Number of machine-code instructions between the two instructions (excluding both).
     */
    static std::size_t countWordsBetween(InstructionWalker fromIt, InstructionWalker toIt)
    {
        std::size_t count = 0;
        for(auto it = fromIt.copy().nextInBlock(); !it.isEndOfBlock() && it != toIt; it.nextInBlock())
        {
            if(it.has() && it->mapsToASMInstruction())
                ++count;
        }
        return count;
    }

    /*
     * Whether combining the two operations keeps all write-read distances of the delay model intact.
     *
     * The combined instruction takes the place of the first operation, the second one is moved up to be its direct
     * successor (secondIsMoved) and its original instruction word disappears, so all instructions after it are one word
     * closer to the combined instruction.
     */
    static bool keepsRegisterDelays(
        InstructionWalker firstIt, InstructionWalker secondIt, bool secondIsMoved, const RegisterMap& registerMap)
    {
        auto firstAccess = getPackAccess(*firstIt.get(), registerMap);
        auto secondAccess = getPackAccess(*secondIt.get(), registerMap);
        if(!firstAccess || !secondAccess)
            return false;
        const bool writtenWithPackMode = firstIt->hasPackMode() || secondIt->hasPackMode();

        // 1) the readers of the values written by the combined instruction
        SortedSet<Register> readers = firstAccess->writes;
        readers.insert(secondAccess->writes.begin(), secondAccess->writes.end());
        bool passedMovedOp = !secondIsMoved;
        for(auto readerIt = firstIt.copy().nextInBlock(); !readerIt.isEndOfBlock() && !readers.empty(); readerIt.nextInBlock())
        {
            if(secondIsMoved && readerIt == secondIt)
            {
                // this operation becomes part of the combined instruction, so it is no reader of it
                passedMovedOp = true;
                continue;
            }
            if(!readerIt.has() || !readerIt->mapsToASMInstruction())
                continue;
            auto access = getPackAccess(*readerIt.get(), registerMap);
            if(!access)
                // unknown access, ignore here - a violation would be reported by verifyRegisterDelays() anyway
                continue;
            // the removed instruction word makes all following instructions one word closer
            const std::size_t distance = countWordsBetween(firstIt, readerIt) + (passedMovedOp ? 0 : 1);
            const bool isRotationOrUnpack = readsWithRotationOrUnpack(*readerIt.get());
            for(auto it = readers.begin(); it != readers.end();)
            {
                if(access->reads.find(*it) == access->reads.end())
                {
                    ++it;
                    continue;
                }
                if(distance < minimumReadDistance(*it, writtenWithPackMode, isRotationOrUnpack))
                    return false;
                it = readers.erase(it);
            }
        }

        // 2) the writers of the values read by the combined instruction
        SortedSet<Register> remaining = firstAccess->reads;
        remaining.insert(secondAccess->reads.begin(), secondAccess->reads.end());
        auto writerIt = firstIt.copy();
        while(!remaining.empty() && !writerIt.isStartOfBlock())
        {
            writerIt.previousInBlock();
            if(!writerIt.has() || !writerIt->mapsToASMInstruction())
                continue;
            auto access = getPackAccess(*writerIt.get(), registerMap);
            if(!access)
                continue;
            const std::size_t distance = countWordsBetween(writerIt, firstIt) + 1;
            for(auto it = remaining.begin(); it != remaining.end();)
            {
                if(access->writes.find(*it) == access->writes.end())
                {
                    ++it;
                    continue;
                }
                if(distance < minimumReadDistance(*it, writerIt->hasPackMode(), false))
                    return false;
                it = remaining.erase(it);
            }
        }

        // 3) if the second operation is moved, the instruction before its original place becomes the direct predecessor
        // of the instruction after it, so a write-read pair spanning exactly that position must not get too close
        if(secondIsMoved)
        {
            auto beforeIt = secondIt.copy();
            while(!beforeIt.isStartOfBlock())
            {
                beforeIt.previousInBlock();
                if(beforeIt.has() && beforeIt->mapsToASMInstruction())
                    break;
            }
            auto afterIt = secondIt.copy().nextInBlock();
            while(!afterIt.isEndOfBlock() && (!afterIt.has() || !afterIt->mapsToASMInstruction()))
                afterIt.nextInBlock();
            if(!beforeIt.isStartOfBlock() && beforeIt.has() && beforeIt->mapsToASMInstruction() && !afterIt.isEndOfBlock())
            {
                auto beforeAccess = getPackAccess(*beforeIt.get(), registerMap);
                auto afterAccess = getPackAccess(*afterIt.get(), registerMap);
                if(beforeAccess && afterAccess)
                {
                    for(const auto& reg : beforeAccess->writes)
                    {
                        if(afterAccess->reads.find(reg) == afterAccess->reads.end())
                            continue;
                        if(minimumReadDistance(reg, beforeIt->hasPackMode(), readsWithRotationOrUnpack(*afterIt.get())) > 1)
                            // the pair would become adjacent and violate its delay rule
                            return false;
                    }
                }
            }
        }

        return true;
    }

    /*
     * Checks whether the moved operation can be moved in front of the given instruction, i.e. whether the two do not
     * access any register in common. Flags, pack/unpack modes and rotations are already excluded by isMovableALUOp().
     */
    static bool canMovePast(const PackAccess& moved, const intermediate::IntermediateInstruction& other,
        const RegisterMap& registerMap)
    {
        auto otherAccess = getPackAccess(other, registerMap);
        if(!otherAccess)
            // unknown instruction, be conservative and do not move across it
            return false;
        return !accessesOverlap(moved, *otherAccess);
    }

    static std::size_t getPackWindowSize()
    {
        static const std::size_t windowSize = []() -> std::size_t {
            if(const char* value = std::getenv("VC4C_PACK_WINDOW"))
            {
                const int parsed = std::atoi(value);
                return parsed > 0 ? static_cast<std::size_t>(parsed) : 0u;
            }
            // a small window, the pairs which can co-issue are usually direct or second-next neighbours
            return 6u;
        }();
        return windowSize;
    }
} // namespace

void optimizations::combineRegisterMappedOperationsWindowed(const Module& module, Method& kernel,
    const Configuration& config, const FastMap<const Local*, Register>& registerMap)
{
    logging::logLazy(logging::Level::DEBUG, [&]() {
        logging::debug() << logging::endl;
        logging::debug() << "Running peephole pass: CombineInstructionsWindowed" << logging::endl;
    });

    PROFILE_COUNTER(
        vc4c::profiler::COUNTER_OPTIMIZATION, "PeepholePackWindowed (before)", kernel.countInstructions());
    PROFILE_START(PeepholePackWindowed);
    std::size_t numChanges = 0;
    std::size_t numPairsChecked = 0;
    const std::size_t windowSize = getPackWindowSize();
    (void) module;
    (void) config;

    if(windowSize >= 2)
    {
        for(auto& block : kernel)
        {
            for(auto it = block.walk(); !it.isEndOfBlock(); it.nextInBlock())
            {
                if(!it.has() || !isMovableALUOp(*it.get(), registerMap))
                    continue;
                auto firstAccess = getPackAccess(*it.get(), registerMap);
                if(!firstAccess)
                    continue;

                auto otherIt = it.copy();
                for(std::size_t distance = 1; distance < windowSize; ++distance)
                {
                    otherIt.nextInBlock();
                    if(otherIt.isEndOfBlock() || !otherIt.has())
                        break;
                    if(!otherIt->mapsToASMInstruction())
                        // a label (or anything else without machine code) must not be crossed
                        break;
                    if(!isMovableALUOp(*otherIt.get(), registerMap))
                        // do not cross anything which is not a plain ALU operation
                        break;
                    ++numPairsChecked;
                    if(!isCombinablePair(*it.get(), *otherIt.get(), registerMap))
                        continue;

                    // check that the operation can be moved in front of all instructions it passes
                    auto secondAccess = getPackAccess(*otherIt.get(), registerMap);
                    bool canMove = secondAccess.has_value();
                    auto betweenIt = it.copy();
                    for(std::size_t passed = 0; canMove && passed + 1 < distance; ++passed)
                    {
                        betweenIt.nextInBlock();
                        while(!betweenIt.isEndOfBlock() && betweenIt != otherIt &&
                            (!betweenIt.has() || !betweenIt->mapsToASMInstruction()))
                            betweenIt.nextInBlock();
                        if(betweenIt == otherIt || betweenIt.isEndOfBlock())
                            break;
                        canMove = canMovePast(*secondAccess, *betweenIt.get(), registerMap);
                    }
                    if(!canMove)
                        continue;
                    // check whether the two operations can be combined at all; those conditions only inspect the two
                    // instructions themselves, so they can be checked before anything is moved
                    MergeConditionData data{&registerMap};
                    auto op = it.get<intermediate::Operation>();
                    auto nextOp = otherIt.get<intermediate::Operation>();
                    auto move = it.get<intermediate::MoveOperation>();
                    auto nextMove = otherIt.get<intermediate::MoveOperation>();
                    if(!std::all_of(MERGE_CONDITIONS.begin(), MERGE_CONDITIONS.end(),
                           [op, nextOp, move, nextMove, &data](
                               const MergeCondition& cond) -> bool { return cond(op, nextOp, move, nextMove, data); }))
                        continue;
                    if(!keepsRegisterDelays(it, otherIt, distance > 1, registerMap))
                    {
                        CPPLOG_LAZY(logging::Level::DEBUG,
                            log << "Not moving '" << otherIt->to_string() << "' in front of '" << it->to_string()
                                << "' (" << distance
                                << " instructions apart): the combination would need an extra delay instruction"
                                << logging::endl);
                        continue;
                    }

                    // all conditions are met, move the operation up (if required) and combine the two
                    auto movedIt = otherIt;
                    if(distance > 1)
                    {
                        CPPLOG_LAZY(logging::Level::DEBUG,
                            log << "Moving '" << otherIt->to_string() << "' " << distance - 1
                                << " instruction(s) up to '" << it->to_string() << logging::endl);
                        auto movedInstruction = otherIt.release();
                        otherIt.erase();
                        movedIt = it.copy().nextInBlock();
                        movedIt.emplace(std::move(movedInstruction));
                    }
                    if(combineOperationsInner(it, movedIt))
                    {
                        CPPLOG_LAZY(logging::Level::DEBUG,
                            log << "Combined operations after moving: " << it->to_string() << logging::endl);
                        ++numChanges;
                    }
                    // in either case, do not look any further for this instruction
                    break;
                }
            }
        }
    }

    PROFILE_END(PeepholePackWindowed);
    PROFILE_COUNTER_WITH_PREV(
        vc4c::profiler::COUNTER_OPTIMIZATION, "PeepholePackWindowed (after)", kernel.countInstructions());
    {
        PROFILE_COUNTER(vc4c::profiler::COUNTER_OPTIMIZATION, "PeepholePackWindowed (changes)", numChanges);
    }
    {
        PROFILE_COUNTER(vc4c::profiler::COUNTER_OPTIMIZATION, "PeepholePackWindowed (pairs)", numPairsChecked);
    }
}
