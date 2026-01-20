#include "epistemic/kripke_model.hpp"
#include "epistemic/formula.hpp"
#include <algorithm>
#include <queue>
#include <stdexcept>

namespace epistemic {

KripkeModel::KripkeModel(const std::set<std::string>& agents)
    : agents_(agents), current_world_("w0") {
    worlds_.insert(current_world_);
    
    for (const auto& agent : agents_) {
        accessibility_[agent][current_world_].insert(current_world_);
    }
}

void KripkeModel::add_world(const std::string& world_id) {
    worlds_.insert(world_id);
}

void KripkeModel::remove_world(const std::string& world_id) {
    worlds_.erase(world_id);
    valuation_.erase(world_id);
    
    for (auto& agent_pair : accessibility_) {
        agent_pair.second.erase(world_id);
        
        for (auto& from_pair : agent_pair.second) {
            from_pair.second.erase(world_id);
        }
    }
    
    if (current_world_ == world_id && !worlds_.empty()) {
        current_world_ = *worlds_.begin();
    }
}

void KripkeModel::add_accessibility_relation(
    const std::string& agent,
    const std::string& from_world,
    const std::string& to_world) {
    
    if (agents_.find(agent) == agents_.end()) {
        throw std::runtime_error("Unknown agent: " + agent);
    }
    
    if (worlds_.find(from_world) == worlds_.end()) {
        throw std::runtime_error("Unknown world: " + from_world);
    }
    
    if (worlds_.find(to_world) == worlds_.end()) {
        throw std::runtime_error("Unknown world: " + to_world);
    }
    
    accessibility_[agent][from_world].insert(to_world);
}

void KripkeModel::set_valuation(
    const std::string& world,
    const std::string& proposition,
    bool value) {
    
    if (worlds_.find(world) == worlds_.end()) {
        throw std::runtime_error("Unknown world: " + world);
    }
    
    valuation_[world][proposition] = value;
}

bool KripkeModel::get_valuation(
    const std::string& world,
    const std::string& proposition) const {
    
    auto world_it = valuation_.find(world);
    if (world_it == valuation_.end()) {
        return false;
    }
    
    auto prop_it = world_it->second.find(proposition);
    if (prop_it == world_it->second.end()) {
        return false;
    }
    
    return prop_it->second;
}

bool KripkeModel::evaluate_atom(
    const std::string& world,
    const std::string& atom) const {
    
    return get_valuation(world, atom);
}

bool KripkeModel::evaluate_knows(
    const std::string& world,
    const std::string& agent,
    const Formula& phi) const {
    
    // K_a(phi) is true at w iff phi is true at all worlds accessible to agent a from w
    auto agent_it = accessibility_.find(agent);
    if (agent_it == accessibility_.end()) {
        return true; // No accessibility relation means vacuously true
    }
    
    auto from_world_it = agent_it->second.find(world);
    if (from_world_it == agent_it->second.end()) {
        return true; // No accessible worlds
    }
    
    const auto& accessible_worlds = from_world_it->second;
    if (accessible_worlds.empty()) {
        return true; // No accessible worlds means vacuously true
    }
    
    for (const auto& accessible_world : accessible_worlds) {
        if (!const_cast<Formula&>(phi).evaluate(*this, accessible_world)) {
            return false;
        }
    }
    
    return true;
}

std::set<std::string> KripkeModel::get_accessible_worlds(
    const std::string& agent,
    const std::string& from_world) const {
    
    auto agent_it = accessibility_.find(agent);
    if (agent_it == accessibility_.end()) {
        return {};
    }
    
    auto world_it = agent_it->second.find(from_world);
    if (world_it == agent_it->second.end()) {
        return {};
    }
    
    return world_it->second;
}

bool KripkeModel::evaluate_common_knowledge(
    const std::string& world,
    const std::set<std::string>& group,
    const Formula& phi) const {
    
    // C_G(phi) = phi holds in all worlds reachable by any sequence of 
    // accessibility relations for agents in the group
    
    std::set<std::string> reachable = get_group_reachable_worlds(world, group);
    
    for (const auto& w : reachable) {
        if (!phi.evaluate(*this, w)) {
            return false;
        }
    }
    
    return true;
}

std::set<std::string> KripkeModel::get_group_reachable_worlds(
    const std::string& start_world,
    const std::set<std::string>& group) const {
    
    std::set<std::string> reachable;
    std::queue<std::string> to_visit;
    
    to_visit.push(start_world);
    reachable.insert(start_world);
    
    while (!to_visit.empty()) {
        std::string current = to_visit.front();
        to_visit.pop();
        
        for (const auto& agent : group) {
            auto accessible = get_accessible_worlds(agent, current);
            
            for (const auto& w : accessible) {
                if (reachable.find(w) == reachable.end()) {
                    reachable.insert(w);
                    to_visit.push(w);
                }
            }
        }
    }
    
    return reachable;
}

void KripkeModel::public_announcement(const Formula& phi) {
    // Public announcement: remove all worlds where phi is false
    std::set<std::string> worlds_to_remove;
    
    for (const auto& world : worlds_) {
        if (!phi.evaluate(*this, world)) {
            worlds_to_remove.insert(world);
        }
    }
    
    for (const auto& world : worlds_to_remove) {
        remove_world(world);
    }
}

void KripkeModel::private_observation(const std::string& agent, const Formula& phi) {
    // Private observation: only the observing agent updates their knowledge
    // Create new worlds for different observations
    // TODO: DEL would use product update
    
    std::set<std::string> worlds_where_phi_true;
    std::set<std::string> worlds_where_phi_false;
    
    for (const auto& world : worlds_) {
        if (phi.evaluate(*this, world)) {
            worlds_where_phi_true.insert(world);
        } else {
            worlds_where_phi_false.insert(world);
        }
    }
    
    auto& agent_accessibility = accessibility_[agent];
    
    for (auto& from_pair : agent_accessibility) {
        std::set<std::string> new_accessible;
        
        bool from_satisfies_phi = (worlds_where_phi_true.find(from_pair.first) != 
                                   worlds_where_phi_true.end());
        
        for (const auto& to_world : from_pair.second) {
            bool to_satisfies_phi = (worlds_where_phi_true.find(to_world) != 
                                     worlds_where_phi_true.end());
            
            // Keep edge only if both satisfy phi or both don't
            if (from_satisfies_phi == to_satisfies_phi) {
                new_accessible.insert(to_world);
            }
        }
        
        from_pair.second = new_accessible;
    }
}

KripkeModel KripkeModel::clone() const {
    KripkeModel cloned(agents_);
    cloned.worlds_ = worlds_;
    cloned.accessibility_ = accessibility_;
    cloned.valuation_ = valuation_;
    cloned.current_world_ = current_world_;
    return cloned;
}

std::vector<std::string> KripkeModel::get_worlds_where(
    const std::string& proposition) const {
    
    std::vector<std::string> result;
    
    for (const auto& world : worlds_) {
        if (evaluate_atom(world, proposition)) {
            result.push_back(world);
        }
    }
    
    return result;
}

} // namespace epistemic