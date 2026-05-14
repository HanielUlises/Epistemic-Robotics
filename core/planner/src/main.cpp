#include "parser.hpp"
#include "validator.hpp"
#include "search.hpp"
#include "heuristic.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <memory>

static void write_plan_tree(std::ostream& out,
                            const std::shared_ptr<PlanNode>& node,
                            int indent = 0) {
    std::string pad(indent * 2, ' ');
    std::string pad2((indent + 1) * 2, ' ');
    std::string pad3((indent + 2) * 2, ' ');

    if (!node) { out << "null"; return; }

    out << "{\n";
    out << pad2 << "\"action\": \"" << node->action << "\",\n";
    out << pad2 << "\"branches\": [\n";

    for (size_t i = 0; i < node->branches.size(); i++) {
        auto& [eid, child] = node->branches[i];
        out << pad3 << "{\n";
        out << pad3 << "  \"event\": " << eid << ",\n";
        out << pad3 << "  \"subtree\": ";
        write_plan_tree(out, child, indent + 3);
        out << "\n" << pad3 << "}";
        if (i + 1 < node->branches.size()) out << ",";
        out << "\n";
    }

    out << pad2 << "]\n";
    out << pad << "}";
}


enum class Strategy { GBFS, EHC, AOSTAR };

static Strategy select_strategy(const PlanningTask& task) {
    // If any action has more than one designated event it is a sensing action —
    // conditional branching is required.
    for (auto& action : task.actions)
        if (action.designated_events.size() > 1)
            return Strategy::AOSTAR;

    // Check whether the goal contains at least one plain atom conjunct.
    // EHC can only make progress when ontic effects directly satisfy goal atoms.
    auto has_atom_conjunct = [](const Formula& f) -> bool {
        if (f.kind == FormulaKind::Atom) return true;
        if (f.kind == FormulaKind::And)
            for (auto& c : f.children)
                if (c->kind == FormulaKind::Atom) return true;
        return false;
    };

    if (has_atom_conjunct(*task.goal)) return Strategy::EHC;

    // Pure epistemic goal (all belief/Kw formulas) — GBFS is safer than EHC
    return Strategy::GBFS;
}

static const char* strategy_name(Strategy s) {
    switch (s) {
        case Strategy::AOSTAR: return "AO* (conditional, auto-selected)";
        case Strategy::EHC:    return "EHC (enforced hill climbing, auto-selected)";
        default:               return "GBFS (auto-selected)";
    }
}


static void usage(const char* prog) {
    std::cerr << "Usage:\n"
              << "  " << prog << " --task <task.json> --plan <plan.json> [options]\n"
              << "\n"
              << "Options:\n"
              << "  --task         Path to ground JSON task (from plank export)\n"
              << "  --plan         Output plan file\n"
              << "  --heuristic    wc = world count (default), ug = unsatisfied goal, ed = epistemic distance\n"
              << "  --limit        Max nodes to expand / max depth (0 = unlimited)\n"
              << "  --ehc          Force Enforced Hill Climbing\n"
              << "  --conditional  Force AND-OR conditional planner\n"
              << "  --gbfs         Force GBFS\n"
              << "  (default: strategy auto-selected from task structure)\n";
}

static void write_linear_plan(std::ostream& out, const SearchResult& result) {
    out << "[";
    for (size_t i = 0; i < result.plan.size(); i++) {
        if (i > 0) out << ", ";
        out << "\"" << result.plan[i] << "\"";
    }
    out << "]\n";
}

int main(int argc, char* argv[]) {
    std::string task_path;
    std::string plan_path;
    std::string heuristic_name = "ug"; 
    size_t      limit          = 0;

    bool force_conditional = false;
    bool force_ehc         = false;
    bool force_gbfs        = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--task" && i + 1 < argc) {
            task_path = argv[++i];
        } else if (arg == "--plan" && i + 1 < argc) {
            plan_path = argv[++i];
        } else if (arg == "--heuristic" && i + 1 < argc) {
            heuristic_name = argv[++i];
        } else if (arg == "--limit" && i + 1 < argc) {
            limit = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--conditional") {
            force_conditional = true;
        } else if (arg == "--ehc") {
            force_ehc = true;
        } else if (arg == "--gbfs") {
            force_gbfs = true;
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            usage(argv[0]);
            return 1;
        }
    }

    if (task_path.empty() || plan_path.empty()) {
        std::cerr << "Error: --task and --plan are required.\n";
        usage(argv[0]);
        return 1;
    }

    PlanningTask task;
    try {
        task = load_task(task_path);
    } catch (const std::exception& e) {
        std::cerr << "Error loading task: " << e.what() << "\n";
        return 1;
    }

    // Heuristic
    std::unique_ptr<Heuristic> h;
    if (heuristic_name == "ug") {
        h = std::make_unique<UnsatisfiedGoalHeuristic>();
        std::cerr << "[main] Heuristic: unsatisfied-goal\n";
    } else if (heuristic_name == "ed") {
        h = std::make_unique<EpistemicDistanceHeuristic>();
        std::cerr << "[main] Heuristic: epistemic-distance\n";
    } else {
        h = std::make_unique<WorldCountHeuristic>();
        std::cerr << "[main] Heuristic: world-count\n";
    }

    // Resolve strategy
    Strategy strategy;
    if (force_conditional)     strategy = Strategy::AOSTAR;
    else if (force_ehc)        strategy = Strategy::EHC;
    else if (force_gbfs)       strategy = Strategy::GBFS;
    else {
        strategy = select_strategy(task);
        std::cerr << "[main] Strategy: " << strategy_name(strategy) << "\n";
    }

    std::ofstream out(plan_path);
    if (!out.is_open()) {
        std::cerr << "Error: cannot open plan file for writing: " << plan_path << "\n";
        return 1;
    }

    if (strategy == Strategy::AOSTAR) {
        std::cerr << "[main] Mode: AO* (conditional plan)\n";

        auto result = aostar::search(task, *h, limit);

        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found — wrote null.\n";
            return 0;
        }

        write_plan_tree(out, result->plan_tree);
        out << "\n";
        std::cerr << "[main] Conditional plan written to " << plan_path << "\n";

        auto vr = validate(task, result->plan_tree);
        if (vr.valid)
            std::cerr << "[validator] OK — "
                      << vr.leaves_reached << " leaves, "
                      << vr.branches_checked << " branches checked\n";
        else
            std::cerr << "[validator] FAILED — " << vr.error << "\n";

    } else if (strategy == Strategy::EHC) {
        std::cerr << "[main] Mode: EHC (enforced hill climbing)\n";

        auto result = ehc::search(task, *h, limit);

        if (!result) {
            // EHC failed (plateau with no escape) — fall back to GBFS
            std::cerr << "[main] EHC failed — falling back to GBFS\n";
            result = gbfs::search(task, *h, limit);
        }

        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found — wrote null.\n";
            return 0;
        }

        write_linear_plan(out, *result);
        std::cerr << "[main] Plan written to " << plan_path << "\n";

    } else {
        std::cerr << "[main] Mode: GBFS\n";

        auto result = gbfs::search(task, *h, limit);

        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found — wrote null.\n";
            return 0;
        }

        write_linear_plan(out, *result);
        std::cerr << "[main] Plan written to " << plan_path << "\n";
    }

    return 0;
}