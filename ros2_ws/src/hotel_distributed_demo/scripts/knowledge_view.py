#!/usr/bin/env python3
# Copyright 2026 Haniel Vásquez Morales
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
What every agent in the hotel knows, after every update.

Everything here is computed from the model the epistemic state publishes
after each product update, at the actual world: the designated world in which
the room named by the `leak` parameter is the source. The other designated
worlds are the rooms the executor cannot yet rule out; no agent's knowledge
is evaluated there.

  each agent   the rooms it cannot rule out, and whether it knows the leak is
               contained, knows it is not, or does not know; and where the
               model has it
  safe         the leak is contained
  stand-down   every responder knows that every responder knows it
  secret       the guest cannot rule out any room

One [knows] line is logged, as JSON, whenever any of these changes; the
film's board and captions are drawn from those lines and from nothing else.
The same JSON is latched on /hotel/knowledge, where the crew reads who may
stand down and the mission reads the verdict.
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
HISTORY = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

AGENTS = ('cleaner', 'concierge', 'porter', 'guest')
RESPONDERS = ('cleaner', 'concierge', 'porter')


class Model:

    def __init__(self, payload):
        self.worlds = [str(w) for w in payload.get('worlds', [])]
        self.designated = [str(w) for w in payload.get('designated', [])]
        self.labels = {str(w): set(a) for w, a in payload.get('labels', {}).items()}
        self.relations = {a: {str(w): [str(v) for v in vs] for w, vs in rel.items()}
                          for a, rel in payload.get('relations', {}).items()}

    def sees(self, agent, world):
        return self.relations.get(agent, {}).get(world, [world])

    def rooms(self):
        return sorted({a[len('source_'):] for w in self.worlds
                       for a in self.labels[w] if a.startswith('source_')})

    def actual(self, leak):
        for w in self.designated:
            if f'source_{leak}' in self.labels.get(w, ()):
                return w
        return None

    def candidates(self, agent, world, rooms):
        return sorted({z for v in self.sees(agent, world) for z in rooms
                       if f'source_{z}' in self.labels.get(v, ())})

    def safe_status(self, agent, world):
        seen = {'safe' in self.labels.get(v, ()) for v in self.sees(agent, world)}
        return 'contained' if seen == {True} else 'leaking' if seen == {False} else 'unsure'

    def everyone_knows(self, world, holds):
        return all(holds(v) for a in RESPONDERS for v in self.sees(a, world))

    def stand_down(self, world):
        """E_R E_R safe at the world."""
        def knows_safe(v):
            return self.everyone_knows(v, lambda u: 'safe' in self.labels.get(u, ()))
        return self.everyone_knows(world, knows_safe)


class KnowledgeView(Node):

    def __init__(self):
        super().__init__('hotel_knowledge')
        self.leak = self.declare_parameter('leak', 'L3_room1').value
        self.fleet = self.declare_parameter('fleet', 'epistemic').value
        self.pub = self.create_publisher(String, '/hotel/knowledge', LATCHED)
        self.create_subscription(String, '/epistemic_state/state', self.on_state, HISTORY)
        self.last = None
        self.updates = 0
        # Every room the run has had in any model. A model after the guest has
        # learnt the room can have a single world, and rooms read off that
        # model alone would make the secret hold of the one room left.
        self.rooms = set()

    def on_state(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if 'model' not in payload:
            return
        model = Model(payload['model'])
        w = model.actual(self.leak)
        if w is None:
            self.get_logger().error(
                f'[knows] no designated world has {self.leak} as the source; the model '
                'and the building disagree')
            return
        self.rooms |= set(model.rooms())
        rooms = sorted(self.rooms)
        agents = {a: {'rooms': model.candidates(a, w, rooms),
                      'safe': model.safe_status(a, w)} for a in AGENTS}
        secret = all(any(f'source_{z}' in model.labels.get(v, ()) for v in model.sees('guest', w))
                     for z in rooms)
        at = {}
        for atom in model.labels[w]:
            if atom.startswith('at-ag_'):
                agent, _, zone = atom[len('at-ag_'):].partition('_')
                at[agent] = zone
        view = {
            'fleet': self.fleet, 'leak': self.leak, 'at': at,
            'worlds': len(model.worlds), 'designated': len(model.designated),
            'executor_rooms': sorted({z for d in model.designated for z in rooms
                                      if f'source_{z}' in model.labels.get(d, ())}),
            'agents': agents,
            'safe': 'safe' in model.labels[w],
            'stand_down': model.stand_down(w),
            'secret': secret,
        }
        key = json.dumps({k: v for k, v in view.items() if k not in ('worlds', 'designated')},
                         sort_keys=True)
        out = json.dumps(view, sort_keys=True)
        self.pub.publish(String(data=out))
        if key != self.last:
            self.last = key
            self.updates += 1
            self.get_logger().info('[knows] ' + out)


def main():
    rclpy.init()
    node = KnowledgeView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
