#pragma once

#include <memory>
#include <string>
#include <variant>

#include "agent.hpp"

namespace epistemic {

/**
 * Atomic proposition (e.g. "cell(3,4) is free").
 * Semantics are defined externally.
 */
struct Atom {
  std::string name;
};

// Forward declaration for recursive formulas.
struct Formula;

// Logical negation.
struct Not {
  std::shared_ptr<Formula> phi;
};

// Logical conjunction.
struct And {
  std::shared_ptr<Formula> left;
  std::shared_ptr<Formula> right;
};

// Knowledge operator: K_a phi
struct Knows {
  Agent agent;
  std::shared_ptr<Formula> phi;
};

// Epistemic formula.
struct Formula {
  using Variant = std::variant<
    Atom,
    Not,
    And,
    Knows
  >;

  Variant value;

  Formula(const Atom& atom) : value(atom) {}
  Formula(const Not& not_f) : value(not_f) {}
  Formula(const And& and_f) : value(and_f) {}
  Formula(const Knows& knows_f) : value(knows_f) {}
};

} // namespace epistemic
