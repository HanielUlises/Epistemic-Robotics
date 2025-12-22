#include "epistemic/atom_interpretation.hpp"

#include <sstream>

namespace epistemic {

bool interpret_atom(
  const World& world,
  const Atom& atom
) {
  const std::string& s = atom.name;

  if (s.rfind("cell_free", 0) == 0) {
    // Parse coordinates
    auto l = s.find('(');
    auto c = s.find(',');
    auto r = s.find(')');

    if (l == std::string::npos ||
        c == std::string::npos ||
        r == std::string::npos) {
      return false;
    }

    int x = std::stoi(s.substr(l + 1, c - l - 1));
    int y = std::stoi(s.substr(c + 1, r - c - 1));

    if (x < 0 || y < 0 ||
        x >= static_cast<int>(world.map.width) ||
        y >= static_cast<int>(world.map.height)) {
      return false;
    }

    return world.map.at(x, y) == CellState::Free;
  }

  // Unknown atom → false
  return false;
}

} // namespace epistemic
