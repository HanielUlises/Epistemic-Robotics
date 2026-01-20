// core/include/epistemic/kripke_model.hpp
#ifndef EPISTEMIC_KRIPKE_MODEL_HPP
#define EPISTEMIC_KRIPKE_MODEL_HPP

#include <string>
#include <set>
#include <map>
#include <vector>
#include <memory>

namespace epistemic {

// Forward declaration
class Formula;

/**
 * @brief Represents a Kripke model for epistemic reasoning
 * 
 * A Kripke model M = (W, R, V) where:
 * - W is a set of possible worlds
 * - R is a set of accessibility relations (one per agent)
 * - V is a valuation function mapping worlds to propositions
 */

struct KripkeModel {
      /**
       * @brief Construct a new Kripke Model with given agents
       * @param agents Set of agent identifiers
       */
      explicit KripkeModel(const std::set<std::string>& agents);
      
      /**
       * @brief Add a new possible world to the model
       * @param world_id Identifier for the world
       */
      void add_world(const std::string& world_id);
      
      /**
       * @brief Remove a world from the model
       * @param world_id Identifier for the world to remove
       */
      void remove_world(const std::string& world_id);
      
      /**
       * @brief Add accessibility relation between worlds for an agent
       * @param agent Agent identifier
       * @param from_world Starting world
       * @param to_world Accessible world
       */
      void add_accessibility_relation(
          const std::string& agent,
          const std::string& from_world,
          const std::string& to_world
      );
      
      /**
       * @brief Set truth value of a proposition in a world
       * @param world World identifier
       * @param proposition Proposition name
       * @param value Truth value
       */
      void set_valuation(
          const std::string& world,
          const std::string& proposition,
          bool value
      );
      
      /**
       * @brief Get truth value of a proposition in a world
       * @param world World identifier
       * @param proposition Proposition name
       * @return true if proposition is true in world, false otherwise
       */
      bool get_valuation(
          const std::string& world,
          const std::string& proposition
      ) const;
      
      /**
       * @brief Evaluate atomic proposition at a world
       * @param world World identifier
       * @param atom Atomic proposition
       * @return true if atom is true at world
       */
      bool evaluate_atom(
          const std::string& world,
          const std::string& atom
      ) const;
      
      /**
       * @brief Evaluate knowledge formula: K_agent(phi)
       * @param world Current world
       * @param agent Agent identifier
       * @param phi Formula that agent might know
       * @return true if agent knows phi at world
       */
      bool evaluate_knows(
          const std::string& world,
          const std::string& agent,
          const Formula& phi
      ) const;
      
      /**
       * @brief Get all worlds accessible to agent from given world
       * @param agent Agent identifier
       * @param from_world Starting world
       * @return Set of accessible world identifiers
       */
      std::set<std::string> get_accessible_worlds(
          const std::string& agent,
          const std::string& from_world
      ) const;
      
      /**
       * @brief Evaluate common knowledge for a group of agents
       * @param world Current world
       * @param group Set of agent identifiers
       * @param phi Formula to check for common knowledge
       * @return true if group has common knowledge of phi at world
       */
      bool evaluate_common_knowledge(
          const std::string& world,
          const std::set<std::string>& group,
          const Formula& phi
      ) const;
      
      /**
       * @brief Get all worlds reachable by group through accessibility relations
       * @param start_world Starting world
       * @param group Set of agent identifiers
       * @return Set of reachable worlds
       */
      std::set<std::string> get_group_reachable_worlds(
          const std::string& start_world,
          const std::set<std::string>& group
      ) const;
      
      /**
       * @brief Apply public announcement (removes worlds where phi is false)
       * @param phi Formula being announced
       */
      void public_announcement(const Formula& phi);
      
      /**
       * @brief Apply private observation by an agent
       * @param agent Agent making the observation
       * @param phi Formula being observed
       */
      void private_observation(const std::string& agent, const Formula& phi);
      
      /**
       * @brief Clone the model
       * @return Deep copy of this model
       */
      KripkeModel clone() const;
      
      /**
       * @brief Get all worlds where proposition is true
       * @param proposition Proposition to check
       * @return Vector of world identifiers
       */
      std::vector<std::string> get_worlds_where(const std::string& proposition) const;
      
      // Getters
      const std::set<std::string>& get_worlds() const { return worlds_; }
      const std::set<std::string>& get_agents() const { return agents_; }
      const std::string& get_current_world() const { return current_world_; }
      void set_current_world(const std::string& world) { current_world_ = world; }
      
  private:
      std::set<std::string> worlds_;
      std::set<std::string> agents_;
      
      // accessibility_[agent][from_world] = set of accessible worlds
      std::map<std::string, std::map<std::string, std::set<std::string>>> accessibility_;
      
      // valuation_[world][proposition] = truth value
      std::map<std::string, std::map<std::string, bool>> valuation_;
      
      std::string current_world_;
};

} // namespace epistemic

#endif // EPISTEMIC_KRIPKE_MODEL_HPP