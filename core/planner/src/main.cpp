#include "parser.hpp"
#include "search.hpp"
#include "heuristic.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <memory>

// Plan tree serialization
//
// Linear plan  -> JSON array:  ["a1", "a2", "a3"]
//
// Conditional plan -> JSON object tree:
// {
//   "action": "move-B-to-A-leave_r2",
//   "branches": [
//     {
//       "event": 0,
//       "subtree": {
//         "action": "pickup-A-hold_r2",
//         "branches": [
//           { "event": 0, "subtree": null }
//         ]
//       }
//     }
//   ]
// }
// A null subtree means that branch already satisfies the goal.

static void write_plan_tree(std::ostream& out,
                            const std::shared_ptr<PlanNode>& node,
                            int indent = 0) {
    std::string pad(indent * 2, ' ');
    std::string pad2((indent + 1) * 2, ' ');
    std::string pad3((indent + 2) * 2, ' ');

    if (!node) {
        out << "null";
        return;
    }

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

static void usage(const char* prog) {
    std::cerr << "Usage:\n"
              << "  " << prog << " --task <task.json> --plan <plan.json> [options]\n"
              << "\n"
              << "Options:\n"
              << "  --task         Path to ground JSON task (from plank export)\n"
              << "  --plan         Output plan file\n"
              << "  --heuristic    wc = world count (default), ug = unsatisfied goal\n"
              << "  --limit        Max nodes to expand / max depth (0 = unlimited)\n"
              << "  --conditional  Use AND-OR conditional planner (default: GBFS)\n";
}

int main(int argc, char* argv[]) {
    std::string task_path;
    std::string plan_path;
    std::string heuristic_name = "wc";
    size_t      limit          = 0;
    bool        conditional    = false;

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
            conditional = true;
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

    // Load task
    PlanningTask task;
    try {
        task = load_task(task_path);
    } catch (const std::exception& e) {
        std::cerr << "Error loading task: " << e.what() << "\n";
        return 1;
    }

    // Select heuristic
    std::unique_ptr<Heuristic> h;
    if (heuristic_name == "ug") {
        h = std::make_unique<UnsatisfiedGoalHeuristic>();
        std::cerr << "[main] Heuristic: unsatisfied-goal\n";
    } else {
        h = std::make_unique<WorldCountHeuristic>();
        std::cerr << "[main] Heuristic: world-count\n";
    }

    // Open output file early
    std::ofstream out(plan_path);
    if (!out.is_open()) {
        std::cerr << "Error: cannot open plan file for writing: " << plan_path << "\n";
        return 1;
    }

    if (conditional) {
        std::cerr << "[main] Mode: conditional (AND-OR, iterative deepening)\n";

        auto result = aostar::search(task, *h, limit);

        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found — wrote null.\n";
            return 0;
        }

        write_plan_tree(out, result->plan_tree);
        out << "\n";
        std::cerr << "[main] Conditional plan written to " << plan_path << "\n";

    } else {
        std::cerr << "[main] Mode: GBFS (linear plan)\n";

        auto result = gbfs::search(task, *h, limit);

        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found — wrote null.\n";
            return 0;
        }

        out << "[";
        for (size_t i = 0; i < result->plan.size(); i++) {
            if (i > 0) out << ", ";
            out << "\"" << result->plan[i] << "\"";
        }
        out << "]\n";
        std::cerr << "[main] Plan written to " << plan_path << "\n";
    }

    return 0;
}