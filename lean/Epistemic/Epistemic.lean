/-
  Epistemic Model for Dynamic Epistemic Logic (DEL)

  This file defines:
  - Agents
  - Worlds
  - Epistemic (Kripke) models
  - Satisfaction relation ⊨

  This is the formal counterpart of your C++ core.
-/

namespace epistemic

/-- Agents are abstract identifiers -/
abbrev Agent := Nat

/-- Atomic propositions are identified by strings -/
abbrev Atom := String

/-- Possible worlds -/
structure World where
  id : Nat
deriving DecidableEq

/-
  Epistemic formulas
-/
inductive Formula where
  | atom  : Atom → Formula
  | not   : Formula → Formula
  | and   : Formula → Formula → Formula
  | knows : Agent → Formula → Formula
deriving DecidableEq

/-
  Epistemic (Kripke) model

  worlds : set of possible worlds
  rel    : epistemic accessibility relation (per agent)
  val    : valuation of atomic propositions
-/
structure EpistemicModel where
  worlds : Set World
  rel    : Agent → World → World → Prop
  val    : World → Atom → Prop

/-
  Satisfaction relation: M, w ⊨ φ
-/
def satisfies (M : EpistemicModel) : World → Formula → Prop
  | w, Formula.atom p =>
      M.val w p

  | w, Formula.not φ =>
      ¬ satisfies M w φ

  | w, Formula.and φ ψ =>
      satisfies M w φ ∧ satisfies M w ψ

  | w, Formula.knows a φ =>
      ∀ w',
        M.rel a w w' →
        satisfies M w' φ

notation:50 M " ⊨[" w "] " φ => satisfies M w φ

/-
  Useful semantic properties
-/

/-- Knowledge is factive: K_a φ → φ (under reflexivity) -/
def reflexive (M : EpistemicModel) (a : Agent) : Prop :=
  ∀ w, M.rel a w w

theorem knowledge_factive
  (M : EpistemicModel)
  (a : Agent)
  (φ : Formula)
  (h_refl : reflexive M a)
  (w : World) :
  M ⊨[w] Formula.knows a φ → M ⊨[w] φ :=
by
  intro h
  apply h
  apply h_refl

/-
  Consistency: a world satisfies some formula
-/
def consistent (M : EpistemicModel) : Prop :=
  ∃ w ∈ M.worlds, True

end epistemic
