/*
 * Author: doe300
 *
 * See the file "LICENSE" for the full license governing this code.
 */

#include "WorkItemAnalysis.h"

#include "../Logger.h"
#include "../Method.h"
#include "../Module.h"
#include "../Profiler.h"
#include "../helper.h"
#include "../intermediate/Helper.h"
#include "ControlFlowGraph.h"
#include "FlagsAnalysis.h"

#include <algorithm>
#include <queue>

using namespace vc4c;
using namespace vc4c::analysis;

bool ActiveWorkItems::isWorkItemCondition() const noexcept
{
    switch(condition)
    {
    case WorkItemCondition::LOCAL_ID_X:
    case WorkItemCondition::LOCAL_ID_Y:
    case WorkItemCondition::LOCAL_ID_Z:
    case WorkItemCondition::LOCAL_ID_SCALAR:
    case WorkItemCondition::GLOBAL_ID_X:
    case WorkItemCondition::GLOBAL_ID_Y:
    case WorkItemCondition::GLOBAL_ID_Z:
        return true;
    default:
        return false;
    }
}

Optional<ActiveWorkItems> ActiveWorkItems::mergeWith(const ActiveWorkItems& other) const
{
    if(condition != other.condition)
    {
        // If the conditions differ, we can never merge.
        CPPLOG_LAZY(logging::Level::DEBUG,
            log << "Cannot merge work-item conditions '" << to_string() << "' and '" << other.to_string()
                << "', assuming no condition!" << logging::endl);
        return {};
    }

    auto result = *this;
    result.activeElements.insert(other.activeElements.begin(), other.activeElements.end());
    result.inactiveElements.insert(other.inactiveElements.begin(), other.inactiveElements.end());

    if(!result.activeElements.empty() && !result.inactiveElements.empty())
    {
        if(result.activeElements == result.inactiveElements)
        {
            // If the included and excluded elements are identical, the conditions does not constraint anything anymore.
            // In words: If one path allows elements A, B and C and the other path allows all elements except A, B and
            // C, together they allow all elements.
            CPPLOG_LAZY(logging::Level::DEBUG,
                log << "Merged work-item conditions '" << to_string() << "' and '" << other.to_string()
                    << "' cancel each other out" << logging::endl);
            return {};
        }

        if(std::includes(result.inactiveElements.begin(), result.inactiveElements.end(), result.activeElements.begin(),
               result.activeElements.end()))
        {
            // If all elements included by one are excluded by the other, remove explicit include/exclude of matching
            // values. In words: If one path allows elements A, B and C and the other path allows all but A, B, C and D,
            // together they allow all but D.
            for(auto element : result.activeElements)
                result.inactiveElements.erase(element);
            result.activeElements.clear();
        }
        else
        {
            // XXX If we have the same condition, but one is exclusive and the other inclusive, we could merge, if we
            // knew the set of all possible values (which we do at least for local IDs).
            // XXX When merging, be pessimistic, add every entry as active, which is at least active in one of the
            // objects!
            CPPLOG_LAZY(logging::Level::DEBUG,
                log << "Cannot merge work-item conditions '" << to_string() << "' and '" << other.to_string()
                    << "', assuming no condition!" << logging::endl);
            return {};
        }
    }

    CPPLOG_LAZY(logging::Level::DEBUG,
        log << "Merged work-item conditions '" << to_string() << "' and '" << other.to_string()
            << "' into: " << result.to_string() << logging::endl);

    return result;
}

LCOV_EXCL_START
static std::string toString(WorkItemCondition cond)
{
    switch(cond)
    {
    case WorkItemCondition::LOCAL_ID_X:
        return "local IDs X";
    case WorkItemCondition::LOCAL_ID_Y:
        return "local IDs Y";
    case WorkItemCondition::LOCAL_ID_Z:
        return "local IDs Z";
    case WorkItemCondition::LOCAL_ID_SCALAR:
        return "local IDs scalar";
    case WorkItemCondition::LOCAL_SIZE_X:
        return "local sizes X";
    case WorkItemCondition::LOCAL_SIZE_Y:
        return "local sizes Y";
    case WorkItemCondition::LOCAL_SIZE_Z:
        return "local sizes Z";
    case WorkItemCondition::LOCAL_SIZE_SCALAR:
        return "local sizes scalar";
    case WorkItemCondition::GLOBAL_ID_X:
        return "global IDs X";
    case WorkItemCondition::GLOBAL_ID_Y:
        return "global IDs Y";
    case WorkItemCondition::GLOBAL_ID_Z:
        return "global IDs Z";
    case WorkItemCondition::GROUP_ID_X:
        return "group IDs X";
    case WorkItemCondition::GROUP_ID_Y:
        return "group IDs Y";
    case WorkItemCondition::GROUP_ID_Z:
        return "group IDs Z";
    }
    throw CompilationError(CompilationStep::GENERAL, "Unhandled work-item condition for basic block",
        std::to_string(static_cast<unsigned>(cond)));
}

std::string ActiveWorkItems::to_string() const
{
    auto toElementString = [](std::size_t element) { return std::to_string(element); };
    auto result = ::toString(condition);
    if(!activeElements.empty())
        result += " included: " + vc4c::to_string<std::size_t>(activeElements, toElementString);
    if(!inactiveElements.empty())
        result += " excluded: " + vc4c::to_string<std::size_t>(inactiveElements, toElementString);
    return result;
}
LCOV_EXCL_STOP

static Optional<ActiveWorkItems> getGlobalMask(const Method& method)
{
    Optional<ActiveWorkItems> globalMask{};
    uint32_t maxSize = 0;
    // TODO additionally set (how to represent??) local_sizes limits!
    if(method.metaData.workGroupSizes[0] >= 1 && method.metaData.workGroupSizes[1] <= 1 &&
        method.metaData.workGroupSizes[2] <= 1)
    {
        globalMask = ActiveWorkItems{WorkItemCondition::LOCAL_ID_X, {}, {}};
        maxSize = method.metaData.workGroupSizes[0];
    }
    else if(method.metaData.workGroupSizes[0] <= 1 && method.metaData.workGroupSizes[1] >= 1 &&
        method.metaData.workGroupSizes[2] <= 1)
    {
        globalMask = ActiveWorkItems{WorkItemCondition::LOCAL_ID_Y, {}, {}};
        maxSize = method.metaData.workGroupSizes[1];
    }
    else if(method.metaData.workGroupSizes[0] <= 1 && method.metaData.workGroupSizes[1] <= 1 &&
        method.metaData.workGroupSizes[2] > 0)
    {
        globalMask = ActiveWorkItems{WorkItemCondition::LOCAL_ID_Z, {}, {}};
        maxSize = method.metaData.workGroupSizes[2];
    }
    else if(auto localSize = method.metaData.getFixedWorkGroupSize())
    {
        globalMask = ActiveWorkItems{WorkItemCondition::LOCAL_ID_SCALAR, {}, {}};
        maxSize = *localSize;
    }
    if(globalMask && maxSize > 0)
    {
        for(uint32_t i = 0; i < maxSize; ++i)
            globalMask->activeElements.emplace(i);
    }
    return globalMask;
}

static Optional<WorkItemCondition> findWorkItemCondition(const Value& value)
{
    auto loc = value.checkLocal();
    if(!loc)
        return {};

    if(auto builtin = loc->as<BuiltinLocal>())
    {
        switch(builtin->builtinType)
        {
        case BuiltinLocal::Type::GROUP_ID_X:
            return WorkItemCondition::GROUP_ID_X;
        case BuiltinLocal::Type::GROUP_ID_Y:
            return WorkItemCondition::GROUP_ID_Y;
        case BuiltinLocal::Type::GROUP_ID_Z:
            return WorkItemCondition::GROUP_ID_Z;
        default:; // handle afterwards
        }
    }

    auto writer = loc->getSingleWriter();
    if(!writer)
        return {};

    if(writer->hasDecoration(intermediate::InstructionDecorations::BUILTIN_LOCAL_ID))
    {
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_X))
            return WorkItemCondition::LOCAL_ID_X;
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_Y))
            return WorkItemCondition::LOCAL_ID_Y;
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_Z))
            return WorkItemCondition::LOCAL_ID_Z;
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_SCALAR))
            return WorkItemCondition::LOCAL_ID_SCALAR;

        auto unpackingInst = dynamic_cast<const intermediate::UnpackingInstruction*>(writer);
        if(unpackingInst && unpackingInst->getUnpackMode() == UNPACK_8A_32)
            return WorkItemCondition::LOCAL_ID_X;
        if(unpackingInst && unpackingInst->getUnpackMode() == UNPACK_8B_32)
            return WorkItemCondition::LOCAL_ID_Y;
        if(unpackingInst && unpackingInst->getUnpackMode() == UNPACK_8C_32)
            return WorkItemCondition::LOCAL_ID_Z;
        return {};
    }
    else if(writer->hasDecoration(intermediate::InstructionDecorations::BUILTIN_LOCAL_SIZE))
    {
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_X))
            return WorkItemCondition::LOCAL_SIZE_X;
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_Y))
            return WorkItemCondition::LOCAL_SIZE_Y;
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_Z))
            return WorkItemCondition::LOCAL_SIZE_Z;
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_SCALAR))
            return WorkItemCondition::LOCAL_SIZE_SCALAR;

        auto unpackingInst = dynamic_cast<const intermediate::UnpackingInstruction*>(writer);
        if(unpackingInst && unpackingInst->getUnpackMode() == UNPACK_8A_32)
            return WorkItemCondition::LOCAL_SIZE_X;
        if(unpackingInst && unpackingInst->getUnpackMode() == UNPACK_8B_32)
            return WorkItemCondition::LOCAL_SIZE_Y;
        if(unpackingInst && unpackingInst->getUnpackMode() == UNPACK_8C_32)
            return WorkItemCondition::LOCAL_SIZE_Z;
        return {};
    }
    else if(writer->hasDecoration(intermediate::InstructionDecorations::BUILTIN_GLOBAL_ID))
    {
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_X))
            return WorkItemCondition::GLOBAL_ID_X;
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_Y))
            return WorkItemCondition::GLOBAL_ID_Y;
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_Z))
            return WorkItemCondition::GLOBAL_ID_Z;

        // global ID is addition of group ID * local size + local ID, so reuse the check for local ID to determine group
        // ID dimensions
        auto op = dynamic_cast<const intermediate::Operation*>(writer);
        if(op && op->op == OP_ADD)
        {
            auto localIdCondition = findWorkItemCondition(op->getFirstArg());
            if(!localIdCondition)
                localIdCondition = findWorkItemCondition(op->assertArgument(1));
            if(localIdCondition)
            {
                switch(*localIdCondition)
                {
                case WorkItemCondition::LOCAL_ID_X:
                case WorkItemCondition::GROUP_ID_X:
                    return WorkItemCondition::GLOBAL_ID_X;
                case WorkItemCondition::LOCAL_ID_Y:
                case WorkItemCondition::GROUP_ID_Y:
                    return WorkItemCondition::GLOBAL_ID_Y;
                case WorkItemCondition::LOCAL_ID_Z:
                case WorkItemCondition::GROUP_ID_Z:
                    return WorkItemCondition::GLOBAL_ID_Z;
                default:
                    return {};
                }
            }
        }
    }
    else if(writer->hasDecoration(intermediate::InstructionDecorations::BUILTIN_GROUP_ID))
    {
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_X))
            return WorkItemCondition::GROUP_ID_X;
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_Y))
            return WorkItemCondition::GROUP_ID_Y;
        if(writer->hasDecoration(intermediate::InstructionDecorations::DIMENSION_Z))
            return WorkItemCondition::GROUP_ID_Z;
    }

    auto source = intermediate::getSourceInstruction(writer);
    if(source && source != writer)
        return findWorkItemCondition(source->getOutput().value());

    return {};
}

static Optional<ComparisonInfo> findDynamicBranchConditon(
    const intermediate::Branch* branch, const Local* targetLabel, InstructionWalker it)
{
    auto loc = branch->getTarget().checkLocal();
    if(!loc)
        return {};
    const intermediate::IntermediateInstruction* matchingWrite = nullptr;
    loc->forUsers(LocalUse::Type::WRITER, [&](const intermediate::IntermediateInstruction* writer) {
        auto addressOf = dynamic_cast<const intermediate::CodeAddress*>(writer);
        if(addressOf && addressOf->getLabel() == targetLabel)
            matchingWrite = writer;
    });

    return matchingWrite ? getComparison(matchingWrite, it, true) : Optional<ComparisonInfo>{};
}

static SortedSet<std::size_t> toElementRange(std::size_t begin, std::size_t end)
{
    SortedSet<std::size_t> result;
    while(begin != end)
    {
        result.emplace(begin);
        ++begin;
    }
    return result;
}

static Optional<ActiveWorkItems> getWorkItemMask(
    const CFGEdge& edge, const CFGNode& entry, const CFGNode& exit, Optional<ActiveWorkItems>&& defaultValue)
{
    auto predecessorBranch = edge.data.getPredecessor(entry.key).get<intermediate::Branch>();
    if(!predecessorBranch)
        // (unconditional) fall-through, copy mask (if exists) of source branch
        return std::move(defaultValue);

    bool invertCondition = edge.data.isImplicit(entry.key) && predecessorBranch->getSingleTargetLabel();
    Optional<ComparisonInfo> comparison;
    if(predecessorBranch->branchCondition == BRANCH_ALWAYS && !predecessorBranch->isDynamicBranch())
        // unconditional branch, copy mask (if exists) of source branch
        return std::move(defaultValue);
    else if(predecessorBranch->isDynamicBranch())
        // try to find the condition for taking the dynamic branch to the current label
        comparison = findDynamicBranchConditon(predecessorBranch, exit.key->getLabel()->getLabel(), entry.key->walk());
    else if(!invertCondition && predecessorBranch->getSingleTargetLabel() != exit.key->getLabel()->getLabel())
        // unsupported branch type, copy mask (if exists) of source branch
        return std::move(defaultValue);
    else
        comparison = getComparison(predecessorBranch, entry.key->walk(), true);
    if(!comparison)
        // unsupported kind of branch condition, cannot deduce comparison, copy mask (if exists) of source branch
        return std::move(defaultValue);

    if(predecessorBranch->getSingleTargetLabel() == exit.key->getLabel()->getLabel() &&
        comparison->condition.isInversionOf(predecessorBranch->branchCondition.toConditionCode()))
        // this is an if-else-branch and the comparison found was for the other target block
        invertCondition = true;

    WorkItemCondition condition;
    Value otherValue = UNDEFINED_VALUE;
    if(auto leftCondition = findWorkItemCondition(comparison->leftOperand))
    {
        condition = *leftCondition;
        otherValue = comparison->rightOperand;
    }
    else if(auto rightCondition = findWorkItemCondition(comparison->rightOperand))
    {
        condition = *rightCondition;
        otherValue = comparison->leftOperand;
    }
    else
        // condition is not depending on work-item information, copy mask (if exists) of source branch
        return std::move(defaultValue);

    auto literalLimit = otherValue.getConstantValue() & &Value::getLiteralValue;
    if(!literalLimit)
        // bounds are not constant, so we don't know how much the condition is limited, since a branch cannot extend the
        // parent bounds, we can safely return them (if set)
        return std::move(defaultValue);

    if((comparison->name == intermediate::COMP_EQ && !invertCondition) ||
        (comparison->name == intermediate::COMP_NEQ && invertCondition))
        // only the single item matches
        return ActiveWorkItems{condition, {literalLimit->unsignedInt()}, {}};
    if((comparison->name == intermediate::COMP_NEQ && !invertCondition) ||
        (comparison->name == intermediate::COMP_EQ && invertCondition))
        // all but the single item match
        return ActiveWorkItems{condition, {}, {literalLimit->unsignedInt()}};

    if(comparison->name == intermediate::COMP_UNSIGNED_LT || comparison->name == intermediate::COMP_SIGNED_LT)
    {
        // A < X <=> A in [0, X - 1]
        auto range = toElementRange(0, literalLimit->unsignedInt());
        return invertCondition ? ActiveWorkItems{condition, {}, std::move(range)} :
                                 ActiveWorkItems{condition, std::move(range), {}};
    }
    if(comparison->name == intermediate::COMP_UNSIGNED_LE || comparison->name == intermediate::COMP_SIGNED_LE)
    {
        // A <= X <=> A in [0, X]
        auto range = toElementRange(0, literalLimit->unsignedInt() + 1u);
        return invertCondition ? ActiveWorkItems{condition, {}, std::move(range)} :
                                 ActiveWorkItems{condition, std::move(range), {}};
    }
    if(comparison->name == intermediate::COMP_UNSIGNED_GT || comparison->name == intermediate::COMP_SIGNED_GT)
    {
        // A > X <=> A not in [0, X]
        auto range = toElementRange(0, literalLimit->unsignedInt() + 1u);
        return invertCondition ? ActiveWorkItems{condition, std::move(range), {}} :
                                 ActiveWorkItems{condition, {}, std::move(range)};
    }
    if(comparison->name == intermediate::COMP_UNSIGNED_GE || comparison->name == intermediate::COMP_SIGNED_GE)
    {
        // A >= X <=> A not in [0, X - 1]
        auto range = toElementRange(0, literalLimit->unsignedInt());
        return invertCondition ? ActiveWorkItems{condition, std::move(range), {}} :
                                 ActiveWorkItems{condition, {}, std::move(range)};
    }

    CPPLOG_LAZY(logging::Level::DEBUG,
        log << "Unhandled case of work-item condition for '" << ::toString(condition) << "' and "
            << literalLimit.to_string() << ": " << comparison->to_string() << logging::endl);
    return {};
}

static bool isWorkItemIdFunction(const std::string& name)
{
    return name == "vc4cl_local_id" || name == "vc4cl_global_id" || name == "vc4cl_local_linear_id" ||
        name == "vc4cl_global_linear_id";
}

/*
 * Finds the method called by the given call-site, mirroring the behavior of the inliner.
 */
static const Method* findCalledMethod(const Module& module, const intermediate::MethodCall& call)
{
    for(const auto& method : module.methods)
    {
        if(call.matchesSignature(*method, true /* exact match */))
            return method.get();
    }
    for(const auto& method : module.methods)
    {
        if(call.matchesSignature(*method, false /* approximate match */))
            return method.get();
    }
    return nullptr;
}

static void scanWorkItemUsage(const Module& module, const Method& method, FastSet<const Method*>& visited,
    bool& usesIds, std::string& reason)
{
    if(!visited.emplace(&method).second)
        return;

    method.forAllInstructions([&](const intermediate::IntermediateInstruction& inst) {
        if(!reason.empty())
            return;

        // barriers are implemented via semaphores, which are indexed by the work-item. They cannot be expressed for
        // 16 work-items running in parallel on the SIMD lanes of one QPU.
        const auto signal = inst.getSignal();
        if(signal == SIGNAL_SWITCH_THREAD || signal == SIGNAL_THREAD_SWITCH_LAST || signal == SIGNAL_END_PROGRAM ||
            signal == SIGNAL_SOFT_BREAK)
        {
            reason = "contains a thread control signal";
            return;
        }

        if(const auto* call = dynamic_cast<const intermediate::MethodCall*>(&inst))
        {
            if(call->methodName == "vc4cl_barrier")
            {
                reason = "contains a work-group barrier";
                return;
            }
            if(isWorkItemIdFunction(call->methodName))
            {
                usesIds = true;
                // the dimension of the ID cannot be checked, if it is not a constant (the value is a constant even if
                // it was propagated through the temporarily created parameter of an inlined function)
                const auto dimension = call->getArgument(0);
                if((call->methodName == "vc4cl_local_id" || call->methodName == "vc4cl_global_id") &&
                    !(dimension && dimension->getConstantValue(true) & &Value::getLiteralValue))
                {
                    reason = "reads a work-item ID with a non-constant dimension: " + call->to_string();
                }
                return;
            }
            if(const auto* calledMethod = findCalledMethod(module, *call))
                scanWorkItemUsage(module, *calledMethod, visited, usesIds, reason);
            return;
        }

        // Work-item-divergent control flow cannot be expressed as a masked store, since the VPM always writes a full
        // row and the DMA copies a fixed depth. Be conservative and reject any conditional branch: even though a
        // branch on a lane-uniform condition could in principle be executed by all merged work-items, the values
        // flowing through such a branch cannot be verified to stay per-lane (see #checkLaneDependentMemoryAccesses).
        if(const auto* branch = dynamic_cast<const intermediate::Branch*>(&inst))
        {
            if(!branch->isUnconditional())
                reason = "contains conditional branches (divergent control flow is not supported)";
        }
    });
}

namespace
{
    /*
     * The change of a value from one merged work-item to the next one (i.e. from one SIMD lane to the following):
     *
     * - "known" with a step of 0 means that all merged work-items calculate the same value
     * - "known" with a non-zero step means the value is a linear function of the work-item ID, e.g. 4 for the byte
     *   address of consecutive elements of a float array
     * - unknown means the value can differ between the merged work-items, but not in a known linear way
     */
    struct LaneStep
    {
        bool known;
        int64_t step;

        static LaneStep uniform()
        {
            return LaneStep{true, 0};
        }

        bool isUniform() const noexcept
        {
            return known && step == 0;
        }
    };

    /*
     * The arithmetic operations for which the lane step can be calculated
     */
    enum class LaneOp
    {
        ADD,
        SUB,
        MUL,
        SHL
    };

    /*
     * Determines how the values of a kernel-function change from one merged work-item to the next.
     *
     * NOTE: This must be run on the kernel-function after inlining, since only then all the data dependencies (also of
     * the inlined functions) are visible.
     */
    class LaneStepAnalysis
    {
    public:
        LaneStep getStep(const Value& value)
        {
            if(value.getConstantValue(true))
                // literals are identical for all merged work-items
                return LaneStep::uniform();

            auto local = value.checkLocal();
            if(local == nullptr)
                // e.g. registers like the element number, which is handled where it is read
                return LaneStep::uniform();

            auto it = results.find(local);
            if(it != results.end())
                return it->second;

            if(!pending.emplace(local).second)
                // circular dependency, assume the worst
                return LaneStep{false, 0};

            auto result = determineStep(*local);
            pending.erase(local);
            results.emplace(local, result);
            return result;
        }

    private:
        FastMap<const Local*, LaneStep> results;
        FastSet<const Local*> pending;

        static LaneStep scale(const LaneStep& step, int64_t factor)
        {
            return step.known ? LaneStep{true, step.step * factor} : step;
        }

        LaneStep determineStep(const Local& local)
        {
            if(local.is<Parameter>() || local.type == TYPE_LABEL)
                // kernel-function arguments (and labels) are identical for all merged work-items
                return LaneStep::uniform();

            bool hasWriter = false;
            LaneStep result = LaneStep::uniform();
            local.forUsers(LocalUse::Type::WRITER, [&](const LocalUser* writer) {
                if(const auto* mem = dynamic_cast<const intermediate::MemoryInstruction*>(writer))
                {
                    if(mem->op != intermediate::MemoryOperation::READ)
                        // the "output" of a memory write is only the address it accesses, not a newly calculated
                        // value
                        return;
                }
                auto writerStep = determineInstructionStep(*writer);
                if(!hasWriter)
                    result = writerStep;
                else if(!writerStep.known || !result.known || writerStep.step != result.step)
                    // multiple definitions with different (or unknown) steps, e.g. at a merge point
                    result = LaneStep{false, 0};
                hasWriter = true;
            });
            if(!hasWriter)
                // e.g. references to built-in locals, which are uniform for the whole work-group
                return LaneStep::uniform();
            return result;
        }

        LaneStep determineInstructionStep(const intermediate::IntermediateInstruction& inst)
        {
            if(inst.readsRegister(REG_ELEMENT_NUMBER))
                // the element number is the index of the SIMD lane, which increments by one per merged work-item
                return LaneStep{true, 1};

            if(const auto* call = dynamic_cast<const intermediate::MethodCall*>(&inst))
            {
                if(isWorkItemIdFunction(call->methodName))
                {
                    // the merged work-items are consecutive along dimension 0, so only the ID of that dimension is
                    // mapped to the SIMD lanes. The IDs of the other dimensions - and all sizes and group IDs - are
                    // identical for all merged work-items.
                    const auto dimension = call->getArgument(0);
                    if(!dimension)
                        return LaneStep{false, 0};
                    const auto literal = dimension->getConstantValue(true);
                    if(!literal)
                        return LaneStep{false, 0};
                    const auto literalValue = literal->getLiteralValue();
                    if(!literalValue)
                        return LaneStep{false, 0};
                    return LaneStep{true, literalValue->unsignedInt() == 0u ? 1 : 0};
                }
                // any other call (e.g. a built-in read of a lane-uniform size) is only uniform if all its arguments are
                return argumentsStep(inst);
            }

            if(const auto* mem = dynamic_cast<const intermediate::MemoryInstruction*>(&inst))
            {
                if(mem->op == intermediate::MemoryOperation::READ)
                    // reading from a lane-uniform address gives the same value for all merged work-items, the value
                    // read from a per-work-item address can be anything
                    return getStep(mem->getSource()).isUniform() ? LaneStep::uniform() : LaneStep{false, 0};
                return LaneStep{false, 0};
            }

            if(const auto* move = dynamic_cast<const intermediate::MoveOperation*>(&inst))
                // simple copies keep the step of their source
                return getStep(move->getSource());

            if(const auto* operation = dynamic_cast<const intermediate::Operation*>(&inst))
            {
                if(operation->op == OP_ADD)
                    return binaryStep(inst, LaneOp::ADD);
                if(operation->op == OP_SUB)
                    return binaryStep(inst, LaneOp::SUB);
                if(operation->op == OP_SHL)
                    return binaryStep(inst, LaneOp::SHL);
                return argumentsStep(inst);
            }
            if(const auto* intrinsic = dynamic_cast<const intermediate::IntrinsicOperation*>(&inst))
            {
                if(intrinsic->opCode == "add")
                    return binaryStep(inst, LaneOp::ADD);
                if(intrinsic->opCode == "sub")
                    return binaryStep(inst, LaneOp::SUB);
                if(intrinsic->opCode == "mul")
                    return binaryStep(inst, LaneOp::MUL);
                if(intrinsic->opCode == "shl")
                    return binaryStep(inst, LaneOp::SHL);
            }

            return argumentsStep(inst);
        }

        /*
         * Fall-back for all other instructions: only exactly reproducible argument values give a reproducible result.
         */
        LaneStep argumentsStep(const intermediate::IntermediateInstruction& inst)
        {
            for(const auto& arg : inst.getArguments())
            {
                if(!getStep(arg).isUniform())
                    return LaneStep{false, 0};
            }
            return LaneStep::uniform();
        }

        LaneStep binaryStep(const intermediate::IntermediateInstruction& inst, LaneOp laneOp)
        {
            const auto& args = inst.getArguments();
            if(args.size() != 2)
                return argumentsStep(inst);
            const auto step0 = getStep(args[0]);
            const auto step1 = getStep(args[1]);

            if(laneOp == LaneOp::ADD || laneOp == LaneOp::SUB)
            {
                if(!step0.known || !step1.known)
                    return LaneStep{false, 0};
                return LaneStep{true, laneOp == LaneOp::ADD ? step0.step + step1.step : step0.step - step1.step};
            }
            if(laneOp == LaneOp::MUL)
            {
                if(!step0.known || !step1.known)
                    return LaneStep{false, 0};
                // a lane-dependent value multiplied by a constant keeps its step, only scaled by the constant
                if(auto literal = args[0].getLiteralValue())
                    return scale(step1, literal->signedInt());
                if(auto literal = args[1].getLiteralValue())
                    return scale(step0, literal->signedInt());
                if(step0.step == 0 && step1.step == 0)
                    return LaneStep::uniform();
                // multiplying by a lane-uniform value which is not a constant gives an unknown step
                return LaneStep{false, 0};
            }
            if(laneOp == LaneOp::SHL)
            {
                if(step0.isUniform())
                    return LaneStep::uniform();
                if(!step0.known)
                    return LaneStep{false, 0};
                if(auto literal = args[1].getLiteralValue())
                    return LaneStep{true, step0.step * static_cast<int64_t>(1ull << (literal->unsignedInt() & 63u))};
                return LaneStep{false, 0};
            }
            return argumentsStep(inst);
        }
    };

    /*
     * Determines the size (in bytes) of the single elements accessed by the given memory instruction.
     */
    static int64_t getAccessedElementBytes(const intermediate::MemoryInstruction& mem)
    {
        const auto elementType = mem.op == intermediate::MemoryOperation::WRITE ? mem.getDestinationElementType() :
                                                                                 mem.getSourceElementType();
        if(!elementType.isSimpleType())
            return 0;
        const auto bitWidth = static_cast<int64_t>(elementType.getScalarBitCount()) *
            static_cast<int64_t>(elementType.getVectorWidth());
        return bitWidth > 0 ? bitWidth / 8 : 0;
    }

    /*
     * The memory lowering always transfers a single contiguous block of elements starting at the address calculated
     * for the first work-item of the merged group (the work-items are consecutive along dimension 0, so this is the
     * first SIMD lane). Thus, a memory access at a lane-dependent address is only correct if the address advances by
     * exactly one accessed element per work-item. Anything else (strided accesses, gathers, ...) cannot be expressed
     * by the hardware and would silently read/write the wrong elements.
     */
    static std::string checkLaneDependentMemoryAccesses(const Method& method, LaneStepAnalysis& laneSteps)
    {
        std::string reason;
        method.forAllInstructions([&](const intermediate::IntermediateInstruction& inst) {
            if(!reason.empty())
                return;
            const auto* mem = dynamic_cast<const intermediate::MemoryInstruction*>(&inst);
            if(mem == nullptr)
                return;

            const bool isWrite = mem->op == intermediate::MemoryOperation::WRITE;
            const auto& address = isWrite ? mem->getDestination() : mem->getSource();
            const auto addressStep = laneSteps.getStep(address);
            if(mem->op == intermediate::MemoryOperation::COPY || mem->op == intermediate::MemoryOperation::FILL)
            {
                // a memory copy/fill only has one address for all merged work-items
                if(!addressStep.isUniform() || !laneSteps.getStep(mem->getSource()).isUniform())
                    reason = "copies memory at a lane-dependent address: " + mem->to_string();
                return;
            }

            if(addressStep.isUniform())
            {
                // all merged work-items access the same address. For a read, the value is simply replicated. For a
                // write, a lane-dependent value cannot be represented, since there is only one element to write.
                if(isWrite && !laneSteps.getStep(mem->getSource()).isUniform())
                    reason = "writes a lane-dependent value to a lane-independent address: " + mem->to_string();
                return;
            }

            const auto elementBytes = getAccessedElementBytes(*mem);
            if(!addressStep.known || elementBytes <= 0 || addressStep.step != elementBytes)
            {
                reason = "accesses memory at a lane-dependent address which does not advance by exactly one element "
                         "per work-item: " +
                    mem->to_string();
            }
        });
        return reason;
    }

    /*
     * Verifies that all lane-dependent values are only used in ways the normalizer can express per work-item, i.e.
     * that the vector width of a lane-dependent value can be propagated to its uses. If this cannot be proven, the
     * kernel must not be merged, since the normalizer would emit a scalar use of a per-lane value.
     */
    static std::string checkLaneDependentValueUses(const Method& method, LaneStepAnalysis& laneSteps)
    {
        std::string reason;
        method.forAllInstructions([&](const intermediate::IntermediateInstruction& inst) {
            if(!reason.empty())
                return;
            if(dynamic_cast<const intermediate::MemoryInstruction*>(&inst) != nullptr)
                // handled by #checkLaneDependentMemoryAccesses
                return;

            bool usesLaneDependent = false;
            for(const auto& arg : inst.getArguments())
            {
                if(!laneSteps.getStep(arg).isUniform())
                {
                    usesLaneDependent = true;
                    break;
                }
            }
            if(!usesLaneDependent)
                return;

            auto out = inst.getOutput();
            if(!out || out->checkLocal() == nullptr)
            {
                reason = "uses a lane-dependent value in a way which cannot be expressed per work-item: " +
                    inst.to_string();
                return;
            }
            if(out->checkLocal()->type == TYPE_BOOL)
                // the vector width propagation does not handle boolean values
                reason = "calculates a lane-dependent boolean value: " + inst.to_string();
        });
        return reason;
    }

    /*
     * The vector width propagation makes every lane-dependent value a 16-element vector, i.e. it assumes one 32-bit
     * element per SIMD lane. Values of a vector type (e.g. a per-work-item float2) or of a 64-bit type would require a
     * different layout, so kernels calculating such values cannot be merged.
     */
    static std::string checkLaneDependentValueTypes(const Method& method, LaneStepAnalysis& laneSteps)
    {
        std::string reason;
        const auto check = [&](const Value& value, const std::string& instruction) {
            if(!reason.empty())
                return;
            auto local = value.checkLocal();
            if(local == nullptr || !local->type.isSimpleType())
                return;
            if(laneSteps.getStep(value).isUniform())
                return;
            if(local->type.getVectorWidth() > 1 || local->type.getScalarBitCount() > 32)
                reason = "calculates a lane-dependent value of more than one 32-bit element: " + instruction;
        };
        method.forAllInstructions([&](const intermediate::IntermediateInstruction& inst) {
            if(!reason.empty())
                return;
            for(const auto& arg : inst.getArguments())
                check(arg, inst.to_string());
            auto out = inst.getOutput();
            if(out)
                check(*out, inst.to_string());
        });
        return reason;
    }
} // namespace

Optional<std::string> analysis::checkWorkItemMerging(const Module& module, const Method& kernel, uint8_t factor)
{
    if(factor <= 1)
        return std::string{"work-item merging is disabled"};

    // the work-items are merged along dimension 0, so the number of work-items in that dimension must be a multiple
    // of the number of SIMD lanes
    if(kernel.metaData.getFixedWorkGroupSize())
    {
        const auto sizeX = kernel.metaData.workGroupSizes[0];
        if(sizeX == 0u || sizeX % factor != 0u)
            return std::string{"the required work-group size is not a multiple of the merge factor"};
    }

    FastSet<const Method*> visited;
    bool usesIds = false;
    std::string reason;
    scanWorkItemUsage(module, kernel, visited, usesIds, reason);
    if(!reason.empty())
        return reason;
    if(!usesIds)
        // without reading any work-item ID, all work-items execute the same code. Merging them would execute the
        // kernel only once per merged group and could change observable behavior (e.g. atomics, global counters).
        return std::string{"does not read any work-item IDs"};

    LaneStepAnalysis laneSteps;
    auto memoryReason = checkLaneDependentMemoryAccesses(kernel, laneSteps);
    if(!memoryReason.empty())
        return memoryReason;
    auto useReason = checkLaneDependentValueUses(kernel, laneSteps);
    if(!useReason.empty())
        return useReason;
    auto typeReason = checkLaneDependentValueTypes(kernel, laneSteps);
    if(!typeReason.empty())
        return typeReason;

    return {};
}

FastMap<const BasicBlock*, ActiveWorkItems> analysis::determineActiveWorkItems(
    const Method& method, ControlFlowGraph& cfg)
{
    PROFILE_SCOPE(determineActiveWorkItems);
    auto globalMask = getGlobalMask(method);

    FastMap<const BasicBlock*, ActiveWorkItems> results;
    // XXX This cannot be a set (even though this would remove duplicates), since if we skip an entry, the same entry
    // might get added to the front back again (e.g. due to sorting/hashing), which would cause an infinite loop
    std::queue<const CFGNode*> openNodes;
    FastSet<const CFGNode*> closedNodes;

    auto processNode = [&](const CFGNode& node) {
        if(closedNodes.find(&node) != closedNodes.end())
            return true;

        bool allPredecessorsHandled = true;
        bool hasIncomingEdges = false;
        node.forAllIncomingEdges([&](const CFGNode& neighbor, const CFGEdge& edge) {
            if(edge.data.isBackEdge(neighbor.key))
                // Ignore back edges, since they cannot modify the active work-items:
                // - A back edge cannot limit the active work-items, since the target block is already accessed
                // initially by more work-items
                // - A back-edge cannot expand the active work-items, since the loop is only visited by the limited
                // number of work-items
                return true;

            hasIncomingEdges = true;
            if(closedNodes.find(&neighbor) == closedNodes.end())
            {
                allPredecessorsHandled = false;
                return false;
            }
            return true;
        });

        if(!allPredecessorsHandled)
        {
            // re-run for this node again after we processed all predecessors
            openNodes.emplace(&node);
            return true;
        }

        Optional<ActiveWorkItems> nodeResult{};
        node.forAllIncomingEdges([&](const CFGNode& neighbor, const CFGEdge& edge) {
            if(edge.data.isBackEdge(neighbor.key))
                return true;

            auto prevResult = globalMask;
            auto resultIt = results.find(neighbor.key);
            if(resultIt != results.end())
                prevResult = resultIt->second;

            if(auto edgeResult = getWorkItemMask(edge, neighbor, node, std::move(prevResult)))
            {
                CPPLOG_LAZY(logging::Level::DEBUG,
                    log << "Found work-item condition for edge '" << neighbor.key->to_string() << " -> "
                        << node.key->to_string() << "': " << edgeResult.to_string() << logging::endl);
                if(!nodeResult)
                    nodeResult = edgeResult;
                else
                    nodeResult = nodeResult->mergeWith(edgeResult.value()) | globalMask;
                // if we manage to create an empty merged result (e.g. non-mergeable conditions), treat as if there were
                // none
                return nodeResult.has_value();
            }
            else
                // could not deduce any work-item limit for this edge -> the node has at least a single entry without a
                // limit
                return false;
        });

        if(!hasIncomingEdges)
            // This is the case for the start block
            nodeResult = globalMask;

        if(nodeResult)
        {
            CPPLOG_LAZY(logging::Level::DEBUG,
                log << "Basic " << node.key->to_string()
                    << " is accessed with work-item condition: " << nodeResult->to_string() << logging::endl);
            results.emplace(node.key, *std::move(nodeResult));
        }

        closedNodes.emplace(&node);

        node.forAllOutgoingEdges([&](const CFGNode& neighbor, const CFGEdge& edge) {
            openNodes.emplace(&neighbor);
            return true;
        });
        return true;
    };

    cfg.forAllSources(processNode);
    while(!openNodes.empty())
    {
        auto* node = openNodes.front();
        openNodes.pop();
        processNode(*node);
    }

#ifndef NDEBUG
    cfg.dumpGraph("/tmp/vc4c-cfg-active-workitems.dot", [&](const BasicBlock* block) {
        auto label = block->getLabel()->getLabel()->name;
        auto blockIt = results.find(block);
        if(blockIt != results.end())
            label += " (" + blockIt->second.to_string() + ")";
        return label;
    });
#endif

    return results;
}
