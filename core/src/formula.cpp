#include "epistemic/formula.hpp"
#include "epistemic/kripke_model.hpp"

namespace epistemic {

// Atom implementation
Atom::Atom(const std::string& proposition) : proposition_(proposition) {}

bool Atom::evaluate(const KripkeModel& model, const std::string& world) const {
    return model.evaluate_atom(world, proposition_);
}

std::unique_ptr<Formula> Atom::clone() const {
    return std::make_unique<Atom>(proposition_);
}

// Not implementation
Not::Not(std::unique_ptr<Formula> subformula) 
    : subformula_(std::move(subformula)) {}

bool Not::evaluate(const KripkeModel& model, const std::string& world) const {
    return !subformula_->evaluate(model, world);
}

std::string Not::to_string() const {
    return "¬(" + subformula_->to_string() + ")";
}

std::unique_ptr<Formula> Not::clone() const {
    return std::make_unique<Not>(subformula_->clone());
}

// And implementation
And::And(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right)
    : left_(std::move(left)), right_(std::move(right)) {}

bool And::evaluate(const KripkeModel& model, const std::string& world) const {
    return left_->evaluate(model, world) && right_->evaluate(model, world);
}

std::string And::to_string() const {
    return "(" + left_->to_string() + " ∧ " + right_->to_string() + ")";
}

std::unique_ptr<Formula> And::clone() const {
    return std::make_unique<And>(left_->clone(), right_->clone());
}

// Or implementation
Or::Or(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right)
    : left_(std::move(left)), right_(std::move(right)) {}

bool Or::evaluate(const KripkeModel& model, const std::string& world) const {
    return left_->evaluate(model, world) || right_->evaluate(model, world);
}

std::string Or::to_string() const {
    return "(" + left_->to_string() + " ∨ " + right_->to_string() + ")";
}

std::unique_ptr<Formula> Or::clone() const {
    return std::make_unique<Or>(left_->clone(), right_->clone());
}

// Implies implementation
Implies::Implies(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right)
    : left_(std::move(left)), right_(std::move(right)) {}

bool Implies::evaluate(const KripkeModel& model, const std::string& world) const {
    return !left_->evaluate(model, world) || right_->evaluate(model, world);
}

std::string Implies::to_string() const {
    return "(" + left_->to_string() + " → " + right_->to_string() + ")";
}

std::unique_ptr<Formula> Implies::clone() const {
    return std::make_unique<Implies>(left_->clone(), right_->clone());
}

// Knows implementation
Knows::Knows(const std::string& agent, std::unique_ptr<Formula> subformula)
    : agent_(agent), subformula_(std::move(subformula)) {}

bool Knows::evaluate(const KripkeModel& model, const std::string& world) const {
    return model.evaluate_knows(world, agent_, *subformula_);
}

std::string Knows::to_string() const {
    return "K_" + agent_ + "(" + subformula_->to_string() + ")";
}

std::unique_ptr<Formula> Knows::clone() const {
    return std::make_unique<Knows>(agent_, subformula_->clone());
}

// CommonKnowledge implementation
CommonKnowledge::CommonKnowledge(
    const std::set<std::string>& group, 
    std::unique_ptr<Formula> subformula)
    : group_(group), subformula_(std::move(subformula)) {}

bool CommonKnowledge::evaluate(const KripkeModel& model, const std::string& world) const {
    return model.evaluate_common_knowledge(world, group_, *subformula_);
}

std::string CommonKnowledge::to_string() const {
    std::string group_str = "{";
    bool first = true;
    for (const auto& agent : group_) {
        if (!first) group_str += ",";
        group_str += agent;
        first = false;
    }
    group_str += "}";
    return "C_" + group_str + "(" + subformula_->to_string() + ")";
}

std::unique_ptr<Formula> CommonKnowledge::clone() const {
    return std::make_unique<CommonKnowledge>(group_, subformula_->clone());
}

// EverybodyKnows implementation
EverybodyKnows::EverybodyKnows(
    const std::set<std::string>& group,
    std::unique_ptr<Formula> subformula)
    : group_(group), subformula_(std::move(subformula)) {}

bool EverybodyKnows::evaluate(const KripkeModel& model, const std::string& world) const {
    // E_G(φ) is true iff K_a(φ) is true for all a in G
    for (const auto& agent : group_) {
        if (!model.evaluate_knows(world, agent, *subformula_)) {
            return false;
        }
    }
    return true;
}

std::string EverybodyKnows::to_string() const {
    std::string group_str = "{";
    bool first = true;
    for (const auto& agent : group_) {
        if (!first) group_str += ",";
        group_str += agent;
        first = false;
    }
    group_str += "}";
    return "E_" + group_str + "(" + subformula_->to_string() + ")";
}

std::unique_ptr<Formula> EverybodyKnows::clone() const {
    return std::make_unique<EverybodyKnows>(group_, subformula_->clone());
}

// Helper functions
std::unique_ptr<Formula> make_atom(const std::string& proposition) {
    return std::make_unique<Atom>(proposition);
}

std::unique_ptr<Formula> make_not(std::unique_ptr<Formula> phi) {
    return std::make_unique<Not>(std::move(phi));
}

std::unique_ptr<Formula> make_and(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right) {
    return std::make_unique<And>(std::move(left), std::move(right));
}

std::unique_ptr<Formula> make_or(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right) {
    return std::make_unique<Or>(std::move(left), std::move(right));
}

std::unique_ptr<Formula> make_implies(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right) {
    return std::make_unique<Implies>(std::move(left), std::move(right));
}

std::unique_ptr<Formula> make_knows(const std::string& agent, std::unique_ptr<Formula> phi) {
    return std::make_unique<Knows>(agent, std::move(phi));
}

std::unique_ptr<Formula> make_common_knowledge(const std::set<std::string>& group, std::unique_ptr<Formula> phi) {
    return std::make_unique<CommonKnowledge>(group, std::move(phi));
}

std::unique_ptr<Formula> make_everybody_knows(const std::set<std::string>& group, std::unique_ptr<Formula> phi) {
    return std::make_unique<EverybodyKnows>(group, std::move(phi));
}

} // namespace epistemic