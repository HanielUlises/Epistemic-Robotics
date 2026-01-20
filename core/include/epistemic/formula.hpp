#ifndef EPISTEMIC_FORMULA_HPP
#define EPISTEMIC_FORMULA_HPP

#include <string>
#include <memory>
#include <set>

namespace epistemic {

class KripkeModel;

enum class FormulaType {
    ATOM,
    NOT,
    AND,
    OR,
    IMPLIES,
    KNOWS,
    COMMON_KNOWLEDGE,
    EVERYBODY_KNOWS
};

/**
 * @brief Base class for epistemic logic formulas
 */
class Formula {
public:
    virtual ~Formula() = default;
    
    /**
     * @brief Evaluate formula at a world in a model
     * @param model The Kripke model
     * @param world The world to evaluate at
     * @return true if formula is satisfied
     */
    virtual bool evaluate(const KripkeModel& model, const std::string& world) const = 0;
    
    virtual FormulaType get_type() const = 0;
    virtual std::string to_string() const = 0;
    virtual std::unique_ptr<Formula> clone() const = 0;
};

/**
 * @brief Atomic proposition
 */
class Atom : public Formula {
public:
    explicit Atom(const std::string& proposition);
    
    bool evaluate(const KripkeModel& model, const std::string& world) const override;
    FormulaType get_type() const override { return FormulaType::ATOM; }
    std::string to_string() const override { return proposition_; }
    std::unique_ptr<Formula> clone() const override;
    
    const std::string& get_proposition() const { return proposition_; }
    
private:
    std::string proposition_;
};

/**
 * @brief Negation: ¬φ
 */
class Not : public Formula {
public:
    explicit Not(std::unique_ptr<Formula> subformula);
    
    bool evaluate(const KripkeModel& model, const std::string& world) const override;
    FormulaType get_type() const override { return FormulaType::NOT; }
    std::string to_string() const override;
    std::unique_ptr<Formula> clone() const override;
    
private:
    std::unique_ptr<Formula> subformula_;
};

/**
 * @brief Conjunction: φ ∧ ψ
 */
class And : public Formula {
public:
    And(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right);
    
    bool evaluate(const KripkeModel& model, const std::string& world) const override;
    FormulaType get_type() const override { return FormulaType::AND; }
    std::string to_string() const override;
    std::unique_ptr<Formula> clone() const override;
    
private:
    std::unique_ptr<Formula> left_;
    std::unique_ptr<Formula> right_;
};

/**
 * @brief Disjunction: φ ∨ ψ
 */
class Or : public Formula {
public:
    Or(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right);
    
    bool evaluate(const KripkeModel& model, const std::string& world) const override;
    FormulaType get_type() const override { return FormulaType::OR; }
    std::string to_string() const override;
    std::unique_ptr<Formula> clone() const override;
    
private:
    std::unique_ptr<Formula> left_;
    std::unique_ptr<Formula> right_;
};

/**
 * @brief Implication: φ → ψ
 */
class Implies : public Formula {
public:
    Implies(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right);
    
    bool evaluate(const KripkeModel& model, const std::string& world) const override;
    FormulaType get_type() const override { return FormulaType::IMPLIES; }
    std::string to_string() const override;
    std::unique_ptr<Formula> clone() const override;
    
private:
    std::unique_ptr<Formula> left_;
    std::unique_ptr<Formula> right_;
};

/**
 * @brief Knowledge operator: K_agent(φ)
 */
class Knows : public Formula {
public:
    Knows(const std::string& agent, std::unique_ptr<Formula> subformula);
    
    bool evaluate(const KripkeModel& model, const std::string& world) const override;
    FormulaType get_type() const override { return FormulaType::KNOWS; }
    std::string to_string() const override;
    std::unique_ptr<Formula> clone() const override;
    
    const std::string& get_agent() const { return agent_; }
    
private:
    std::string agent_;
    std::unique_ptr<Formula> subformula_;
};

/**
 * @brief Common knowledge: C_Group(φ)
 */
class CommonKnowledge : public Formula {
public:
    CommonKnowledge(const std::set<std::string>& group, std::unique_ptr<Formula> subformula);
    
    bool evaluate(const KripkeModel& model, const std::string& world) const override;
    FormulaType get_type() const override { return FormulaType::COMMON_KNOWLEDGE; }
    std::string to_string() const override;
    std::unique_ptr<Formula> clone() const override;
    
private:
    std::set<std::string> group_;
    std::unique_ptr<Formula> subformula_;
};

/**
 * @brief Everybody knows: E_Group(φ)
 * E_G(φ) means every agent in G knows φ
 */
class EverybodyKnows : public Formula {
public:
    EverybodyKnows(const std::set<std::string>& group, std::unique_ptr<Formula> subformula);
    
    bool evaluate(const KripkeModel& model, const std::string& world) const override;
    FormulaType get_type() const override { return FormulaType::EVERYBODY_KNOWS; }
    std::string to_string() const override;
    std::unique_ptr<Formula> clone() const override;
    
private:
    std::set<std::string> group_;
    std::unique_ptr<Formula> subformula_;
};

std::unique_ptr<Formula> make_atom(const std::string& proposition);
std::unique_ptr<Formula> make_not(std::unique_ptr<Formula> phi);
std::unique_ptr<Formula> make_and(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right);
std::unique_ptr<Formula> make_or(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right);
std::unique_ptr<Formula> make_implies(std::unique_ptr<Formula> left, std::unique_ptr<Formula> right);
std::unique_ptr<Formula> make_knows(const std::string& agent, std::unique_ptr<Formula> phi);
std::unique_ptr<Formula> make_common_knowledge(const std::set<std::string>& group, std::unique_ptr<Formula> phi);
std::unique_ptr<Formula> make_everybody_knows(const std::set<std::string>& group, std::unique_ptr<Formula> phi);

} // namespace epistemic

#endif // EPISTEMIC_FORMULA_HPP