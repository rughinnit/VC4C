/*
 * See the file "LICENSE" for the full license governing this code.
 */

#include "RegisterDelays.h"

#include "CompilationError.h"

#include "../InstructionWalker.h"
#include "../Method.h"
#include "../Module.h"
#include "../Profiler.h"
#include "../Register.h"
#include "../asm/ALUInstruction.h"
#include "../asm/BranchInstruction.h"
#include "../asm/Instruction.h"
#include "../asm/LoadInstruction.h"
#include "../asm/SemaphoreInstruction.h"
#include "../intermediate/IntermediateInstruction.h"
#include "log.h"

#include <algorithm>
#include <sstream>

using namespace vc4c;
using namespace vc4c::optimizations;
using namespace vc4c::qpu_asm;

namespace
{
    /*
     * ==================================================================================================================
     * The delay model
     * ==================================================================================================================
     *
     * The rules below are taken from the VideoCore IV 3D Architecture Reference Guide (VideoCoreIV-AG100-R, revision
     * 2013-09-16). Page numbers are the printed page numbers of that document.
     *
     * A distance of 1 means the reading instruction may directly follow the writing one, a distance of 2 means exactly
     * one instruction must be placed in between, and so on.
     *
     * Summary of the rules which apply to a register write and a later read of the same register:
     *
     * - Writing a register in one of the two general-purpose register files and reading it in the next instruction
     *   returns the previous value: "As the QPU pipeline length from register file read to write-back is greater than
     *   four cycles, one cannot write data to a physical QPU register-file in one instruction and then read that same
     *   data for use in the next instruction (no forwarding paths are provided)" (page 18). The same restriction is
     *   repeated in the summary on page 37.
     * - The accumulators are exempt from that rule: "QPU code is expected to make heavy use of the six accumulator
     *   registers, which do not suffer the restriction that data written by an instruction cannot be read in the next
     *   instruction" (page 18).
     * - A vector rotation is executed in the MUL branch of the pipeline, which has an additional stage ("And Color
     *   Convert / Vector Rotate") in front of the write-back (Figure 2 "QPU Core Pipeline", page 17, with the footnote
     *   "Vector Rotate is partially in Accumulator addressing and partially in MUL 2 stage"). The result of a rotation
     *   is therefore written one stage later than the result of a regular ALU operation and must not be read in the
     *   next instruction, even if it is written to an accumulator.
     * - The rotated operand itself is read earlier than a regular operand: "An instruction that does a vector rotate
     *   must not immediately follow an instruction that writes to the accumulator that is being rotated" and "An
     *   instruction that does a vector rotate by r5 must not immediately follow an instruction that writes to r5"
     *   (page 37).
     * - A read with an unpack mode and a write with a pack mode access the pack/unpack blocks in front of (unpack) or
     *   behind (pack) the ALUs (Figure 2, page 17), which adds a stage for the value to be stable. Since pack modes
     *   can only be encoded for register-file A (see Operation#convertToAsm()), a packed write is additionally covered
     *   by the rule for the general-purpose register files above.
     * - Some peripheral (signalling) instructions write accumulator r4 and make the data available to the instruction
     *   following them: "Data in r4 from the hardware is then available to the instruction following the one which
     *   signalled the read" (page 19) and, for the tile buffer, "On the write phase of the signaling instruction, the
     *   relevant tile buffer data arrives and is written to accumulator r4. The following instruction can then use the
     *   data" (page 24).
     * - An SFU lookup writes its result to r4 and needs more time: "Instruction N+0 Write to SFU, N+1 Don't touch r4,
     *   N+2 Don't touch r4, N+3 Instruction can read r4 (SFU result)" (page 23).
     * - Writing the uniforms address register requires the two following instructions to not access uniforms: "there
     *   must be at least two nonuniform-accessing instructions following a pointer change before uniforms can be
     *   accessed once more" (page 22).
     * - "If TMU_NOSWAP is written, the write must be three instructions before the first TMU write instruction"
     *   (page 37).
     * - VPM read data is available to the third instruction after the read setup ("read data is available to read
     *   after a minimum latency of three QPU instructions", page 56). VC4C relies on the VPM read stalling instead
     *   (see the comment at Register#REG_VPM_IO), which is why the minimum distance here is 1. The row exists so the
     *   rule is written down and checked, not because a delay is ever inserted for it.
     * - Branches always execute the three following instructions ("three 'delay slot' instructions following a branch
     *   instruction are always executed", page 20). This is structural: the delay slots are never re-ordered or
     *   removed by the passes here, which is why no instruction is ever inserted into them.
     *
     * The rules above ONLY describe when the hardware needs a specific number of instructions in between a write and a
     * read. Everything else (the NOPs for the TMU/SFU/VPM handshakes, the register-file selection, the register
     * allocation itself) is unchanged and handled by the existing passes.
     */

    enum class WriteKind : unsigned char
    {
        ANY,
        /* result of an operation on the add or mul ALU, written at the end of the regular ALU pipeline */
        ALU,
        /* result of an ALU operation with a horizontal vector rotation */
        ROTATION,
        /* result of an ALU operation with a pack mode, always written to register-file A */
        PACKED,
        /* peripheral load signalling data into accumulator r4 (TMU/TLB load) */
        R4_LOAD,
        /* SFU lookup, result written into accumulator r4 */
        SFU_RESULT,
        /* write to the uniforms address register */
        UNIFORMS_ADDRESS,
        /* write to the TMU_NOSWAP register */
        TMU_NOSWAP,
        /* write to the VPM read setup register */
        VPM_SETUP
    };

    enum class DestinationKind : unsigned char
    {
        ANY,
        /* one of the general-purpose register files A and B */
        GENERAL_PURPOSE,
        /* one of the accumulators r0-r3 and r5 */
        ACCUMULATOR,
        /* accumulator r4, written by peripheral units only */
        R4,
        /* any other register (periphery, no register) */
        OTHER
    };

    enum class ReaderKind : unsigned char
    {
        ANY,
        /* regular register read by an ALU operation */
        ALU,
        /* source operand of a horizontal vector rotation */
        ROTATION,
        /* read with an unpack mode */
        UNPACK,
        /* read of the uniforms stream */
        UNIFORM,
        /* read of accumulator r4 */
        R4,
        /* access to the TMU address/config registers */
        TMU_ADDRESS,
        /* read of the VPM data register */
        VPM_DATA
    };

    struct DelayRule
    {
        WriteKind writer;
        DestinationKind destination;
        ReaderKind reader;
        std::size_t minDistance;
        /* where the rule comes from, so that it can be re-checked against the manual instead of being taken on faith */
        const char* citation;
    };

    /*
     * The minimum distance between writing a register and reading it again, in instruction slots.
     */
    static constexpr DelayRule DELAY_RULES[] = {
        {WriteKind::ALU, DestinationKind::GENERAL_PURPOSE, ReaderKind::ANY, 2,
            "page 18: no forwarding paths from a general-purpose register-file write to a read in the next "
            "instruction; page 37: \"An instruction must not read from a location in physical regfile A or B that was "
            "written to by the previous instruction.\""},
        {WriteKind::ALU, DestinationKind::ACCUMULATOR, ReaderKind::ANY, 1,
            "page 18: the accumulators \"do not suffer the restriction that data written by an instruction cannot be "
            "read in the next instruction\""},
        /*
         * NOTE: For the result of a vector rotation there is no separate row, so the rule for the destination register
         * file above applies. The list of instruction restrictions on page 37 does not contain a rule for reading a
         * rotated result either, and a kernel which reads the rotated value in the very next instruction of a loop
         * (gemm_v16) computes correct results. See the M18 report for the measurement which at first looked like such
         * a rule and turned out to have a different cause.
         */
        {WriteKind::PACKED, DestinationKind::ANY, ReaderKind::ANY, 2,
            "page 17, Figure 2: the pack block sits between the ALUs and the write-back of register-file A, adding a "
            "stage before the value is stable"},
        {WriteKind::ANY, DestinationKind::ANY, ReaderKind::ROTATION, 2,
            "page 37: \"An instruction that does a vector rotate must not immediately follow an instruction that writes "
            "to the accumulator that is being rotated.\" and \"... must not immediately follow an instruction that "
            "writes to r5.\""},
        {WriteKind::ANY, DestinationKind::ANY, ReaderKind::UNPACK, 2,
            "page 17, Figure 2: the unpack block sits between the register-file read and the ALUs, so the value must "
            "be stable one instruction earlier than for a regular read"},
        {WriteKind::R4_LOAD, DestinationKind::R4, ReaderKind::ANY, 1,
            "page 19: \"Data in r4 from the hardware is then available to the instruction following the one which "
            "signalled the read\" (page 24 for the tile buffer)"},
        {WriteKind::SFU_RESULT, DestinationKind::R4, ReaderKind::ANY, 3,
            "page 23: \"Instruction N+3 Instruction can read r4 (SFU result)\""},
        {WriteKind::UNIFORMS_ADDRESS, DestinationKind::ANY, ReaderKind::UNIFORM, 3,
            "page 22: \"there must be at least two nonuniform-accessing instructions following a pointer change before "
            "uniforms can be accessed once more\""},
        {WriteKind::TMU_NOSWAP, DestinationKind::ANY, ReaderKind::TMU_ADDRESS, 3,
            "page 37: \"If TMU_NOSWAP is written, the write must be three instructions before the first TMU write "
            "instruction.\""},
        {WriteKind::VPM_SETUP, DestinationKind::ANY, ReaderKind::VPM_DATA, 1,
            "page 56: read data is available three instructions after the read setup, but VC4C relies on the VPM read "
            "stalling (see the comment at Register#REG_VPM_IO)"},
    };

    static std::size_t requiredDistance(
        const WriteKind writer, const DestinationKind destination, const ReaderKind reader)
    {
        // unless a rule says otherwise, a value is available to the very next instruction
        std::size_t distance = 1;
        for(const auto& rule : DELAY_RULES)
        {
            if(rule.writer != WriteKind::ANY && rule.writer != writer)
                continue;
            if(rule.destination != DestinationKind::ANY && rule.destination != destination)
                continue;
            if(rule.reader != ReaderKind::ANY && rule.reader != reader)
                continue;
            distance = std::max(distance, rule.minDistance);
        }
        return distance;
    }

    static const char* citationFor(
        const WriteKind writer, const DestinationKind destination, const ReaderKind reader, const std::size_t distance)
    {
        const char* result = "page 18: no forwarding paths between the register files and the ALUs";
        for(const auto& rule : DELAY_RULES)
        {
            if(rule.minDistance != distance || rule.minDistance < 2)
                continue;
            if(rule.writer != WriteKind::ANY && rule.writer != writer)
                continue;
            if(rule.destination != DestinationKind::ANY && rule.destination != destination)
                continue;
            if(rule.reader != ReaderKind::ANY && rule.reader != reader)
                continue;
            result = rule.citation;
        }
        return result;
    }

    /*
     * ==================================================================================================================
     * Instruction analysis
     * ==================================================================================================================
     */

    /* A write to one of these registers starts a restriction for the instructions following it, spanning a register
     * which is not the written one, so it cannot be checked as a simple write-read pair on a single register.
     */
    enum class RestrictedAccess : unsigned char
    {
        SFU_R4,
        TMU_NOSWAP,
        UNIFORMS_POINTER
    };

    struct StateRule
    {
        WriteKind setter;
        RestrictedAccess restricted;
        std::size_t minDistance;
        const char* citation;
    };

    static constexpr StateRule STATE_RULES[] = {
        {WriteKind::SFU_RESULT, RestrictedAccess::SFU_R4, 3,
            "page 23: \"Instruction N+1 Don't touch r4, Instruction N+2 Don't touch r4, Instruction N+3 Instruction can "
            "read r4 (SFU result)\""},
        {WriteKind::TMU_NOSWAP, RestrictedAccess::TMU_NOSWAP, 3,
            "page 37: \"If TMU_NOSWAP is written, the write must be three instructions before the first TMU write "
            "instruction.\""},
        {WriteKind::UNIFORMS_ADDRESS, RestrictedAccess::UNIFORMS_POINTER, 3,
            "page 22: \"there must be at least two nonuniform-accessing instructions following a pointer change before "
            "uniforms can be accessed once more\""},
    };

    struct WriteAccess
    {
        Register reg;
        WriteKind kind;
        DestinationKind destination;
    };

    struct ReadAccess
    {
        Register reg;
        ReaderKind kind;
    };

    struct InstructionAccess
    {
        FastAccessList<WriteAccess> writes;
        FastAccessList<ReadAccess> reads;
        /* set if the instruction triggers one of the state restrictions above */
        FastAccessList<WriteKind> setters;
        /* which restricted accesses this instruction performs */
        FastAccessList<RestrictedAccess> restricted;
    };

    static bool isNoRegister(const Register& reg)
    {
        // address 39 in all register files is the "no register" used for unused operands and results
        return reg.num == REG_NOP.num;
    }

    static bool isR4(const Register& reg)
    {
        return reg.file == RegisterFile::ACCUMULATOR && reg.num == REG_TMU_OUT.num;
    }

    static DestinationKind toDestinationKind(const Register& reg)
    {
        if(isR4(reg))
            return DestinationKind::R4;
        // NOTE: isAccumulator() only checks the register number, since the accumulators r0-r3/r5 are addressed as
        // 32-37 in both register files
        if(reg.isGeneralPurpose())
            return DestinationKind::GENERAL_PURPOSE;
        if(reg.isAccumulator())
            return DestinationKind::ACCUMULATOR;
        return DestinationKind::OTHER;
    }

    static WriteKind toWriteKind(const Register& reg, const WriteKind instructionKind)
    {
        if(reg.isSpecialFunctionsUnit())
            return WriteKind::SFU_RESULT;
        if(reg.num == REG_UNIFORM_ADDRESS.num)
            return WriteKind::UNIFORMS_ADDRESS;
        if(reg.num == REG_TMU_NOSWAP.num && reg.file != RegisterFile::ACCUMULATOR)
            return WriteKind::TMU_NOSWAP;
        if(reg.num == REG_VPM_IN_SETUP.num || reg.num == REG_VPM_OUT_SETUP.num)
            return WriteKind::VPM_SETUP;
        return instructionKind;
    }

    static ReaderKind toReaderKind(const Register& reg, const ReaderKind instructionKind, bool& accessesTmu,
        bool& readsUniform)
    {
        if(isR4(reg))
            return ReaderKind::R4;
        if(reg.num == REG_UNIFORM.num && reg.file != RegisterFile::ACCUMULATOR)
        {
            readsUniform = true;
            return ReaderKind::UNIFORM;
        }
        if(reg.isTextureMemoryUnit())
        {
            accessesTmu = true;
            return ReaderKind::TMU_ADDRESS;
        }
        if(reg.num == REG_VPM_IO.num)
            return ReaderKind::VPM_DATA;
        return instructionKind;
    }

    static void addWrite(InstructionAccess& access, const Register& reg, const WriteKind instructionKind)
    {
        if(isNoRegister(reg))
            return;
        const auto kind = toWriteKind(reg, instructionKind);
        access.writes.push_back(WriteAccess{reg, kind, toDestinationKind(reg)});
        if(kind == WriteKind::SFU_RESULT || kind == WriteKind::TMU_NOSWAP ||
            kind == WriteKind::UNIFORMS_ADDRESS)
            access.setters.push_back(kind);
        if(reg.isTextureMemoryUnit())
            access.restricted.push_back(RestrictedAccess::TMU_NOSWAP);
    }

    static void addRead(InstructionAccess& access, const Register& reg, const ReaderKind instructionKind)
    {
        if(isNoRegister(reg))
            return;
        bool accessesTmu = false;
        bool readsUniform = false;
        const auto kind = toReaderKind(reg, instructionKind, accessesTmu, readsUniform);
        access.reads.push_back(ReadAccess{reg, kind});
        if(isR4(reg))
            access.restricted.push_back(RestrictedAccess::SFU_R4);
        if(accessesTmu)
            access.restricted.push_back(RestrictedAccess::TMU_NOSWAP);
        if(readsUniform)
            access.restricted.push_back(RestrictedAccess::UNIFORMS_POINTER);
    }

    static InstructionAccess analyseInstruction(const Instruction& instr)
    {
        InstructionAccess access;
        if(auto branch = instr.as<BranchInstruction>())
        {
            // a branch writes the link address into both register-file outputs if it has an output, otherwise both
            // outputs are the "no register" and are dropped by addWrite()
            addWrite(access, branch->getAddOutput(), WriteKind::ALU);
            addWrite(access, branch->getMulOutput(), WriteKind::ALU);
            if(branch->getAddRegister() == BranchReg::BRANCH_REG)
                // the branch target is read from register-file A
                addRead(access, Register{RegisterFile::PHYSICAL_A, branch->getRegisterAddress()}, ReaderKind::ALU);
            return access;
        }
        if(instr.as<SemaphoreInstruction>())
            // semaphores do not access any register
            return access;
        if(auto load = instr.as<LoadInstruction>())
        {
            addWrite(access, load->getAddOutput(), WriteKind::ALU);
            addWrite(access, load->getMulOutput(), WriteKind::ALU);
            return access;
        }
        auto alu = instr.as<ALUInstruction>();
        if(!alu)
            return access;

        WriteKind writeKind = WriteKind::ALU;
        if(alu->isVectorRotation())
            writeKind = WriteKind::ROTATION;
        else if(alu->getPack() != PACK_NOP)
            writeKind = WriteKind::PACKED;
        addWrite(access, alu->getAddOutput(), writeKind);
        addWrite(access, alu->getMulOutput(), writeKind);

        ReaderKind readKind = ReaderKind::ALU;
        if(alu->getUnpack() != UNPACK_NOP && alu->getUnpack() != UNPACK_NOP_PM)
            readKind = ReaderKind::UNPACK;
        if(alu->isVectorRotation())
        {
            // The offset of a rotation is not a register read, it is encoded in the input fields, so only the first
            // operand is the rotated value. Only for a dynamic offset the value is read from r5.
            addRead(access, alu->getMulFirstOperand(), ReaderKind::ROTATION);
            if(alu->getMulSecondOperand().num == REG_ACC5.num)
                addRead(access, REG_ACC5, ReaderKind::ROTATION);
        }
        else
        {
            // NOTE: for small immediates (including the rotations above), the B input field does not address a
            // register but encodes the immediate value, which is why the accessors return the "no register" for them
            addRead(access, alu->getAddFirstOperand(), readKind);
            addRead(access, alu->getAddSecondOperand(), readKind);
            addRead(access, alu->getMulFirstOperand(), readKind);
            addRead(access, alu->getMulSecondOperand(), readKind);
        }

        if(instr.getSig().triggersReadOfR4())
            // the signalling instruction itself provides the data in r4 to the following instruction
            addWrite(access, REG_TMU_OUT, WriteKind::R4_LOAD);
        return access;
    }

    /*
     * ==================================================================================================================
     * Applying the model to an instruction sequence
     * ==================================================================================================================
     */

    struct Violation
    {
        std::size_t index;
        std::string description;
    };

    static const char* to_string(const WriteKind kind)
    {
        switch(kind)
        {
        case WriteKind::ANY:
            return "any write";
        case WriteKind::ALU:
            return "ALU result";
        case WriteKind::ROTATION:
            return "vector rotation result";
        case WriteKind::PACKED:
            return "packed ALU result";
        case WriteKind::R4_LOAD:
            return "peripheral load into r4";
        case WriteKind::SFU_RESULT:
            return "SFU lookup";
        case WriteKind::UNIFORMS_ADDRESS:
            return "uniforms address write";
        case WriteKind::TMU_NOSWAP:
            return "TMU_NOSWAP write";
        case WriteKind::VPM_SETUP:
            return "VPM read setup write";
        }
        return "write";
    }

    static const char* to_string(const ReaderKind kind)
    {
        switch(kind)
        {
        case ReaderKind::ANY:
            return "any read";
        case ReaderKind::ALU:
            return "ALU read";
        case ReaderKind::ROTATION:
            return "vector rotation source";
        case ReaderKind::UNPACK:
            return "unpacking read";
        case ReaderKind::UNIFORM:
            return "uniform read";
        case ReaderKind::R4:
            return "r4 read";
        case ReaderKind::TMU_ADDRESS:
            return "TMU access";
        case ReaderKind::VPM_DATA:
            return "VPM data read";
        }
        return "read";
    }

    /*
     * Computes the number of instructions which must be inserted in front of each instruction to satisfy the delay
     * model, and reports every violation which cannot be fixed by inserting instructions (because the position of the
     * instruction is dictated by the control flow, e.g. for branch delay slots).
     *
     * If #mayInsert is false for an instruction, no instruction is inserted in front of it, so all violations are
     * reported. Running the analysis that way on the final machine code is the verifier.
     */
    static FastAccessList<Violation> computeRequiredDelays(const FastAccessList<InstructionAccess>& accesses,
        const FastAccessList<bool>& mayInsert, FastAccessList<std::size_t>& numInstructionsBefore)
    {
        FastAccessList<Violation> violations;
        numInstructionsBefore.assign(accesses.size(), 0);

        FastMap<Register, std::pair<std::size_t, std::pair<WriteKind, DestinationKind>>> lastWrites;
        FastMap<WriteKind, std::size_t> setterSlots;
        std::size_t slot = 0;

        for(std::size_t i = 0; i < accesses.size(); ++i)
        {
            const auto& access = accesses[i];
            std::size_t needed = 0;
            std::ostringstream description;
            while(true)
            {
                needed = 0;
                description.str("");
                description.clear();
                for(const auto& restricted : access.restricted)
                {
                    for(const auto& rule : STATE_RULES)
                    {
                        if(rule.restricted != restricted)
                            continue;
                        auto it = setterSlots.find(rule.setter);
                        if(it == setterSlots.end())
                            continue;
                        const auto actual = slot - it->second;
                        if(actual < rule.minDistance && rule.minDistance - actual > needed)
                        {
                            needed = rule.minDistance - actual;
                            description << (restricted == RestrictedAccess::SFU_R4 ? "accesses r4" : "accesses the restricted register")
                                        << " " << actual << " instruction(s) after the " << to_string(rule.setter)
                                        << ", " << rule.minDistance << " required (" << rule.citation << ")";
                        }
                    }
                }
                for(const auto& read : access.reads)
                {
                    auto it = lastWrites.find(read.reg);
                    if(it == lastWrites.end())
                        continue;
                    const auto required =
                        requiredDistance(it->second.second.first, it->second.second.second, read.kind);
                    const auto actual = slot - it->second.first;
                    if(actual < required && required - actual > needed)
                    {
                        needed = required - actual;
                        description.str("");
                        description.clear();
                        description << "reads " << read.reg.to_string() << " (" << to_string(read.kind) << ") " << actual
                                    << " instruction(s) after the " << to_string(it->second.second.first)
                                    << " writing it, " << required << " required ("
                                    << citationFor(it->second.second.first, it->second.second.second, read.kind,
                                           required)
                                    << ")";
                    }
                }
                if(needed == 0)
                    break;
                if(i >= mayInsert.size() || !mayInsert[i])
                {
                    // cannot fix this by inserting instructions here, report it
                    violations.push_back(Violation{i, description.str()});
                    break;
                }
                numInstructionsBefore[i] += needed;
                slot += needed;
                CPPLOG_LAZY(logging::Level::DEBUG,
                    log << "Inserting " << needed << " delay instruction(s) in front of instruction " << i << ", which "
                        << description.str() << logging::endl);
            }

            for(const auto& write : access.writes)
                lastWrites[write.reg] = std::make_pair(slot, std::make_pair(write.kind, write.destination));
            for(const auto setter : access.setters)
                setterSlots[setter] = slot;
            ++slot;
        }
        return violations;
    }

    static FastAccessList<InstructionAccess> analyseSequence(const FastAccessList<Instruction>& instructions)
    {
        FastAccessList<InstructionAccess> accesses;
        accesses.reserve(instructions.size());
        for(const auto& instruction : instructions)
            accesses.push_back(analyseInstruction(instruction));
        return accesses;
    }

    /*
     * An instruction may not be moved into the three delay slots following a branch, and the last three instructions of
     * a kernel are the thread-end sequence. Instructions must never be inserted in front of those.
     */
    static bool canInsertInstructionsBefore(const FastAccessList<Instruction>& instructions, const std::size_t index)
    {
        for(std::size_t back = 1; back <= 3 && back <= index; ++back)
        {
            if(instructions[index - back].as<BranchInstruction>())
                return false;
        }
        return index + 3 < instructions.size();
    }

    struct Slot
    {
        InstructionWalker walker;
        Instruction instruction;

        Slot(InstructionWalker&& walker, const Instruction& instruction) :
            walker(std::move(walker)), instruction(instruction)
        {
        }
    };

    static FastAccessList<Slot> collectSlots(Method& kernel, const FastMap<const Local*, Register>& registerMap)
    {
        // the label mapping is not needed for the analysis, use a dummy so the conversion does not fail
        FastMap<const Local*, std::size_t> labelMapping;
        auto labelIt = kernel.walkAllInstructions();
        while(!labelIt.isEndOfMethod())
        {
            if(auto label = labelIt.get<intermediate::BranchLabel>())
                labelMapping.emplace(label->getLabel(), 0u);
            labelIt.nextInMethod();
        }

        FastAccessList<Slot> slots;
        for(auto& block : kernel)
        {
            auto it = block.walk();
            while(!it.isEndOfBlock())
            {
                if(it.has() && it->mapsToASMInstruction())
                {
                    auto mapped = it->convertToAsm(registerMap, labelMapping, slots.size());
                    slots.emplace_back(it.copy(), mapped.instruction);
                }
                it.nextInBlock();
            }
        }
        return slots;
    }
} // namespace

void optimizations::insertMissingRegisterDelays(const Module& module, Method& kernel, const Configuration& config,
    const FastMap<const Local*, Register>& registerMap)
{
    PROFILE_SCOPE(insertMissingRegisterDelays);

    auto slots = collectSlots(kernel, registerMap);
    if(slots.empty())
        return;

    FastAccessList<Instruction> instructions;
    instructions.reserve(slots.size());
    for(std::size_t i = 0; i < slots.size(); ++i)
        instructions.push_back(slots[i].instruction);

    FastAccessList<bool> mayInsert;
    mayInsert.reserve(slots.size());
    for(std::size_t i = 0; i < slots.size(); ++i)
        mayInsert.push_back(canInsertInstructionsBefore(instructions, i));

    const auto accesses = analyseSequence(instructions);
    FastAccessList<std::size_t> numInstructionsBefore;
    const auto violations = computeRequiredDelays(accesses, mayInsert, numInstructionsBefore);

    std::size_t numInserted = 0;
    for(std::size_t i = 0; i < numInstructionsBefore.size(); ++i)
    {
        for(std::size_t n = 0; n < numInstructionsBefore[i]; ++n)
        {
            slots[i].walker.emplace(std::make_unique<intermediate::Nop>(intermediate::DelayType::WAIT_REGISTER));
            ++numInserted;
        }
    }

    for(const auto& violation : violations)
    {
        // the verifier will turn this into a compilation error, this only helps to find the cause
        logging::warn() << "Cannot insert a delay instruction in front of instruction " << violation.index
                        << " of kernel '" << kernel.name << "', which " << violation.description << logging::endl;
    }
    if(numInserted > 0)
    {
        CPPLOG_LAZY(logging::Level::DEBUG,
            log << "Inserted " << numInserted << " delay instructions for the register delay model into kernel '"
                << kernel.name << "'" << logging::endl);
        PROFILE_COUNTER(vc4c::profiler::COUNTER_BACKEND, "Register delay instructions inserted", numInserted);
    }
    (void) module;
    (void) config;
}

void optimizations::verifyRegisterDelays(const Module& module, Method& kernel,
    const FastAccessList<qpu_asm::DecoratedInstruction>& instructions)
{
    PROFILE_SCOPE(verifyRegisterDelays);

    FastAccessList<Instruction> machineCode;
    machineCode.reserve(instructions.size());
    for(const auto& instruction : instructions)
        machineCode.push_back(instruction.instruction);

    const auto accesses = analyseSequence(machineCode);
    // no instruction may be inserted anymore at this point, so every violation is reported
    FastAccessList<bool> mayInsert(machineCode.size(), false);
    FastAccessList<std::size_t> numInstructionsBefore;
    const auto violations = computeRequiredDelays(accesses, mayInsert, numInstructionsBefore);

    if(!violations.empty())
    {
        std::ostringstream message;
        message << "The generated machine-code for kernel '" << kernel.name
                << "' violates the QPU register delay model (the hardware would silently use the previous value)";
        std::size_t index = 0;
        for(const auto& violation : violations)
        {
            if(index++ == 5)
            {
                message << ", ...";
                break;
            }
            message << ", instruction " << violation.index << " " << violation.description;
        }
        throw CompilationError(CompilationStep::CODE_GENERATION, message.str());
    }
    (void) module;
}
